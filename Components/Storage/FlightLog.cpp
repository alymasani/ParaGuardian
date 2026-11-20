/**
 ******************************************************************************
 * @file    FlightLog.cpp
 * @brief   See FlightLog.hpp.
 ******************************************************************************
 */
#include "Storage/FlightLog.hpp"

#include <cstring>

#include "Algorithms/Emergency/SosRequest.hpp"   // crc32Ieee

namespace pg {

namespace {
constexpr uint32_t kPagesPerSector = static_cast<uint32_t>(partition::kSector / logfmt::kPageBytes);

void put32(uint8_t* d, uint32_t v)
{
    for (size_t i = 0; i < 4u; ++i) d[i] = static_cast<uint8_t>(v >> (8u * i));
}
uint32_t get32(const uint8_t* s)
{
    uint32_t v = 0u;
    for (size_t i = 0; i < 4u; ++i) v |= static_cast<uint32_t>(s[i]) << (8u * i);
    return v;
}
bool allErased(const uint8_t* p, size_t n)
{
    for (size_t i = 0; i < n; ++i) {
        if (p[i] != 0xFFu) return false;
    }
    return true;
}
void inc(uint32_t& c, uint32_t by = 1u)
{
    c = (c > UINT32_MAX - by) ? UINT32_MAX : c + by;
}

/// Parses the records of one page buffer from @p start. Returns where parsing stopped.
template <typename F>
size_t parsePage(const uint8_t* page, size_t start, F&& onRecord)
{
    size_t off = start;
    while (off < logfmt::kPageBytes && page[off] != 0xFFu) {
        LogEntry e{};
        uint32_t seq = 0u;
        const size_t n = decodeLogRecord(&page[off], logfmt::kPageBytes - off, e, seq);
        if (n == 0u) break;   // torn / garbage: the rest of this page is not trusted
        onRecord(e, seq);
        off += n;
    }
    return off;
}
}  // namespace

FlightLog::FlightLog(INorFlash& flash, const FlashRegion& region, uint8_t tag, LogFullPolicy policy)
    : flash_(flash), region_(region), tag_(tag), policy_(policy)
{
}

bool FlightLog::readSectorHeader(INorFlash& flash, uint32_t address, uint8_t tag, uint32_t& seq)
{
    uint8_t h[logfmt::kSectorHeaderBytes];
    if (!isOk(flash.read(address, h, sizeof(h)))) return false;
    if (get32(&h[0]) != logfmt::kSectorMagic || h[4] != logfmt::kSectorVersion || h[5] != tag) return false;
    if (get32(&h[12]) != crc32Ieee(h, 12u)) return false;
    seq = get32(&h[8]);
    return seq != 0u && seq != UINT32_MAX;
}

LogMount FlightLog::mount()
{
    stats_.recoveredRecords = stats_.tornAtMount = stats_.invalidSectors = 0u;
    full_ = false;
    needOpen_ = true;
    if (!flash_.ready() || region_.sectors < 2u || (region_.base % partition::kSector) != 0u) {
        mount_ = LogMount::Unavailable;
        return mount_;
    }
    uint32_t best = 0u, bestSector = 0u, reads = 0u;
    bool anyData = false;
    for (uint32_t s = 0; s < region_.sectors; ++s) {
        uint32_t seq = 0u;
        if (readSectorHeader(flash_, sectorAddress(s), tag_, seq)) {
            ++reads;
            if (seq > best) {
                best = seq;
                bestSector = s;
            }
            continue;
        }
        uint8_t h[logfmt::kSectorHeaderBytes];
        if (isOk(flash_.read(sectorAddress(s), h, sizeof(h)))) {
            ++reads;
            if (!allErased(h, sizeof(h))) {
                anyData = true;
                inc(stats_.invalidSectors);
            }
        }
    }
    if (reads == 0u) {
        mount_ = LogMount::Unavailable;
        return mount_;
    }
    if (best == 0u) {
        mount_ = anyData ? LogMount::Corrupt : LogMount::Blank;
        sector_ = 0u;
        sectorSeq_ = 0u;   // first openSector() writes sequence 1
        nextSeq_ = 1u;
        return mount_;
    }
    sector_ = bestSector;
    sectorSeq_ = best;
    resumeInHead();
    mount_ = LogMount::Recovered;
    return mount_;
}

void FlightLog::resumeInHead()
{
    uint32_t maxSeq = 0u;
    bool any = false;
    uint32_t resumePage = 0u;
    size_t resumeOff = logfmt::kSectorHeaderBytes;
    for (uint32_t p = 0; p < kPagesPerSector; ++p) {
        uint8_t buf[logfmt::kPageBytes];
        if (!isOk(flash_.read(sectorAddress(sector_) + p * logfmt::kPageBytes, buf, sizeof(buf)))) {
            resumePage = p + 1u;   // unknown content: never append into it
            resumeOff = 0u;
            continue;
        }
        const size_t start = p == 0u ? logfmt::kSectorHeaderBytes : 0u;
        if (allErased(&buf[start], logfmt::kPageBytes - start)) continue;
        const size_t end = parsePage(buf, start, [&](const LogEntry&, uint32_t seq) {
            inc(stats_.recoveredRecords);
            if (!any || seq > maxSeq) maxSeq = seq;
            any = true;
        });
        if (end < logfmt::kPageBytes && allErased(&buf[end], logfmt::kPageBytes - end)) {
            resumePage = p;
            resumeOff = end;
            std::memcpy(pageBuf_, buf, sizeof(pageBuf_));
        } else {
            if (end < logfmt::kPageBytes) inc(stats_.tornAtMount);
            resumePage = p + 1u;
            resumeOff = 0u;
        }
    }
    nextSeq_ = any ? maxSeq + 1u : 1u;
    page_ = resumePage;
    pageOff_ = flushedOff_ = resumeOff;
    pageRecords_ = 0u;
    if (resumeOff == 0u) std::memset(pageBuf_, 0xFF, sizeof(pageBuf_));
    needOpen_ = false;
    if (page_ >= kPagesPerSector) {   // head full: the next append rotates
        page_ = kPagesPerSector - 1u;
        pageOff_ = flushedOff_ = logfmt::kPageBytes;
    }
}

Status FlightLog::openSector(uint32_t s, uint32_t seq)
{
    needOpen_ = true;
    Status st = flash_.eraseSector(sectorAddress(s));
    for (uint32_t p = 0; isOk(st) && p < kPagesPerSector; ++p) {   // blank check: an erase that did not finish
        uint8_t buf[logfmt::kPageBytes];
        st = flash_.read(sectorAddress(s) + p * logfmt::kPageBytes, buf, sizeof(buf));
        if (isOk(st) && !allErased(buf, sizeof(buf))) st = Status::VerifyFailed;
    }
    if (!isOk(st)) {
        inc(stats_.eraseFailures);
        sector_ = s;
        sectorSeq_ = seq - 1u;   // retried by the next append
        return st;
    }
    inc(stats_.sectorsErased);
    std::memset(pageBuf_, 0xFF, sizeof(pageBuf_));
    put32(&pageBuf_[0], logfmt::kSectorMagic);
    pageBuf_[4] = logfmt::kSectorVersion;
    pageBuf_[5] = tag_;
    pageBuf_[6] = pageBuf_[7] = 0u;
    put32(&pageBuf_[8], seq);
    put32(&pageBuf_[12], crc32Ieee(pageBuf_, 12u));
    st = flash_.program(sectorAddress(s), pageBuf_, logfmt::kSectorHeaderBytes);
    uint32_t check = 0u;
    if (isOk(st) && !readSectorHeader(flash_, sectorAddress(s), tag_, check)) st = Status::VerifyFailed;
    if (!isOk(st)) {
        inc(stats_.programFailures);
        sector_ = s;
        sectorSeq_ = seq - 1u;
        return st;
    }
    if (sectorSeq_ != 0u) inc(stats_.rotations);
    sector_ = s;
    sectorSeq_ = seq;
    page_ = 0u;
    pageOff_ = flushedOff_ = logfmt::kSectorHeaderBytes;
    pageRecords_ = 0u;
    needOpen_ = false;
    return Status::Ok;
}

Status FlightLog::flush()
{
    if (needOpen_ || pageOff_ == flushedOff_) return Status::Ok;
    const uint32_t addr = sectorAddress(sector_) + page_ * logfmt::kPageBytes + static_cast<uint32_t>(flushedOff_);
    const size_t n = pageOff_ - flushedOff_;
    Status st = flash_.program(addr, &pageBuf_[flushedOff_], n);
    uint8_t check[logfmt::kPageBytes];
    if (isOk(st)) st = flash_.read(addr, check, n);
    if (isOk(st) && std::memcmp(check, &pageBuf_[flushedOff_], n) != 0) st = Status::VerifyFailed;
    if (!isOk(st)) {
        inc(stats_.programFailures);
        inc(stats_.recordsLost, pageRecords_);
        pageRecords_ = 0u;
        // The page may now hold partial bits: abandon it, continue on the next page.
        pageOff_ = flushedOff_ = logfmt::kPageBytes;
        return st;
    }
    inc(stats_.programs);
    inc(stats_.bytesProgrammed, static_cast<uint32_t>(n));
    flushedOff_ = pageOff_;
    pageRecords_ = 0u;
    return Status::Ok;
}

Status FlightLog::advancePage()
{
    const Status fl = flush();
    if (page_ + 1u < kPagesPerSector) {
        ++page_;
        pageOff_ = flushedOff_ = 0u;
        std::memset(pageBuf_, 0xFF, sizeof(pageBuf_));
        return fl;
    }
    const uint32_t next = (sector_ + 1u) % region_.sectors;
    uint32_t seq = 0u;
    if (policy_ == LogFullPolicy::StopWhenFull && readSectorHeader(flash_, sectorAddress(next), tag_, seq)) {
        full_ = true;   // the next sector still holds older records: keep them
        return Status::BufferOverflow;
    }
    const Status op = openSector(next, sectorSeq_ + 1u);
    return isOk(op) ? fl : op;
}

Status FlightLog::append(const LogEntry& e)
{
    if (mount_ == LogMount::NotMounted || mount_ == LogMount::Unavailable) return Status::NotInitialized;
    if (full_) {
        inc(stats_.droppedFull);
        return Status::BufferOverflow;
    }
    if (needOpen_) {
        const Status st = openSector(sector_, sectorSeq_ + 1u);
        if (!isOk(st)) return st;
    }
    uint8_t rec[kLogMaxRecord];
    const size_t n = encodeLogRecord(e, nextSeq_, rec, sizeof(rec));
    if (n == 0u) return Status::InvalidArgument;
    if (pageOff_ + n > logfmt::kPageBytes) {
        const Status st = advancePage();
        if (full_) {
            inc(stats_.droppedFull);
            return Status::BufferOverflow;
        }
        if (needOpen_) return isOk(st) ? Status::Busy : st;   // rotation erase failed: retry later
    }
    std::memcpy(&pageBuf_[pageOff_], rec, n);
    pageOff_ += n;
    ++nextSeq_;
    ++pageRecords_;
    inc(stats_.recordsAppended);
    return Status::Ok;
}

size_t FlightLog::scan(INorFlash& flash, const FlashRegion& region, uint8_t tag, Visitor visit, void* ctx)
{
    if (region.sectors == 0u) return 0u;
    uint32_t headSeq = 0u, head = 0u;
    for (uint32_t s = 0; s < region.sectors; ++s) {
        uint32_t seq = 0u;
        if (readSectorHeader(flash, region.base + s * partition::kSector, tag, seq) && seq > headSeq) {
            headSeq = seq;
            head = s;
        }
    }
    if (headSeq == 0u) return 0u;
    size_t visited = 0u;
    // Ring order: oldest sector is the one after the head.
    for (uint32_t k = 1; k <= region.sectors; ++k) {
        const uint32_t s = (head + k) % region.sectors;
        const uint32_t base = region.base + s * partition::kSector;
        uint32_t seq = 0u;
        if (!readSectorHeader(flash, base, tag, seq) || seq > headSeq) continue;
        for (uint32_t p = 0; p < kPagesPerSector; ++p) {
            uint8_t buf[logfmt::kPageBytes];
            if (!isOk(flash.read(base + p * logfmt::kPageBytes, buf, sizeof(buf)))) continue;
            parsePage(buf, p == 0u ? logfmt::kSectorHeaderBytes : 0u, [&](const LogEntry& e, uint32_t rs) {
                if (visit != nullptr) visit(ctx, e, rs);
                ++visited;
            });
        }
    }
    return visited;
}

}  // namespace pg
