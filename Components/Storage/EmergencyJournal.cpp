/**
 ******************************************************************************
 * @file    EmergencyJournal.cpp
 * @brief   See EmergencyJournal.hpp.
 ******************************************************************************
 */
#include "Storage/EmergencyJournal.hpp"

#include <cstring>

#include "Algorithms/Emergency/SosRequest.hpp"   // crc32Ieee

namespace pg {

namespace {
constexpr uint32_t kMaxSectors = 8u;
constexpr uint32_t kNone = UINT32_MAX;

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
bool erased(const uint8_t* p, size_t n)
{
    for (size_t i = 0; i < n; ++i) {
        if (p[i] != 0xFFu) return false;
    }
    return true;
}
void inc(uint32_t& c)
{
    if (c != UINT32_MAX) ++c;
}
}  // namespace

EmergencyJournal::EmergencyJournal(INorFlash& flash, const FlashRegion& region) : flash_(flash), region_(region)
{
    const NorGeometry g = flash.geometry();
    slotsPerSector_ = g.sectorSize / journal::kSlotBytes;
}

uint32_t EmergencyJournal::slotAddress(uint32_t sector, uint32_t slot) const
{
    return region_.base + sector * partition::kSector + slot * static_cast<uint32_t>(journal::kSlotBytes);
}

void EmergencyJournal::encodeSlot(uint8_t* s, uint32_t seq, uint32_t boot, const uint8_t* payload, uint32_t uptimeMs)
{
    std::memset(s, 0, journal::kSlotBytes);
    put32(&s[0], journal::kMagic);
    s[4] = journal::kLayoutVersion;
    s[5] = journal::kTypeEmergency;
    s[6] = static_cast<uint8_t>(journal::kPayloadBytes);
    put32(&s[8], seq);
    put32(&s[12], boot);
    std::memcpy(&s[16], payload, journal::kPayloadBytes);
    put32(&s[48], uptimeMs);
    put32(&s[journal::kCrcOffset], crc32Ieee(s, journal::kCrcOffset));
    put32(&s[journal::kCommitOffset], journal::kCommitWord);
}

int EmergencyJournal::classifySlot(const uint8_t* s, uint32_t& seq)
{
    if (get32(&s[0]) != journal::kMagic || s[4] != journal::kLayoutVersion || s[5] != journal::kTypeEmergency ||
        s[6] != journal::kPayloadBytes) {
        return 0;
    }
    if (get32(&s[journal::kCrcOffset]) != crc32Ieee(s, journal::kCrcOffset)) return 0;
    seq = get32(&s[8]);
    if (seq == 0u || seq == UINT32_MAX) return 0;
    return get32(&s[journal::kCommitOffset]) == journal::kCommitWord ? 2 : 1;
}

JournalMount EmergencyJournal::mount()
{
    inc(stats_.mounts);
    stats_.validAtMount = stats_.tornAtMount = stats_.uncommittedAtMount = 0u;
    haveLatest_ = haveUncommitted_ = false;
    latestSector_ = kNone;
    if (!flash_.ready() || region_.sectors < 2u || region_.sectors > kMaxSectors || slotsPerSector_ == 0u ||
        (region_.base % partition::kSector) != 0u) {
        mount_ = JournalMount::Unavailable;
        return mount_;
    }
    uint32_t lastUsed[kMaxSectors];
    uint32_t bestSeq = 0u, bestSlot = 0u, bestBoot = 0u, uncSeq = 0u, readOk = 0u, nonErased = 0u;
    for (uint32_t sec = 0; sec < region_.sectors; ++sec) {
        lastUsed[sec] = kNone;
        for (uint32_t i = 0; i < slotsPerSector_; ++i) {
            if (!isOk(flash_.read(slotAddress(sec, i), slot_, sizeof(slot_)))) {
                inc(stats_.readFailures);
                lastUsed[sec] = i;   // unknown content: never write here without an erase
                continue;
            }
            ++readOk;
            if (erased(slot_, sizeof(slot_))) continue;
            ++nonErased;
            lastUsed[sec] = i;
            uint32_t seq = 0u;
            const int c = classifySlot(slot_, seq);
            if (c == 2) {
                inc(stats_.validAtMount);
                if (seq > bestSeq) {
                    bestSeq = seq;
                    bestBoot = get32(&slot_[12]);
                    latestSector_ = sec;
                    bestSlot = i;
                    std::memcpy(latest_, &slot_[16], sizeof(latest_));
                    haveLatest_ = true;
                }
            } else if (c == 1) {
                inc(stats_.uncommittedAtMount);
                if (seq > uncSeq) {
                    uncSeq = seq;
                    std::memcpy(uncommitted_, &slot_[16], sizeof(uncommitted_));
                    haveUncommitted_ = true;
                }
            } else {
                inc(stats_.tornAtMount);
            }
        }
    }
    (void)bestSlot;
    if (readOk == 0u) {
        mount_ = JournalMount::Unavailable;
        haveLatest_ = haveUncommitted_ = false;
        return mount_;
    }
    if (haveUncommitted_ && uncSeq < bestSeq) haveUncommitted_ = false;   // older than the committed state
    seq_ = bestSeq > uncSeq ? bestSeq : uncSeq;                             // never reuse any intact sequence
    boot_ = haveLatest_ ? (bestBoot == UINT32_MAX ? bestBoot : bestBoot + 1u) : 1u;
    if (haveLatest_) {
        mount_ = JournalMount::Recovered;
        cursorSector_ = latestSector_;
    } else {
        mount_ = nonErased == 0u ? JournalMount::Blank : JournalMount::Corrupt;
        cursorSector_ = 0u;
    }
    // Append after the last non-erased slot of the active sector (skips torn slots).
    cursorSlot_ = lastUsed[cursorSector_] == kNone ? 0u : lastUsed[cursorSector_] + 1u;
    const uint32_t next = (cursorSector_ + 1u) % region_.sectors;
    nextErased_ = lastUsed[next] == kNone;
    return mount_;
}

bool EmergencyJournal::latest(uint8_t* out, size_t len) const
{
    if (!haveLatest_ || out == nullptr || len != journal::kPayloadBytes) return false;
    std::memcpy(out, latest_, len);
    return true;
}

bool EmergencyJournal::uncommitted(uint8_t* out, size_t len) const
{
    if (!haveUncommitted_ || out == nullptr || len != journal::kPayloadBytes) return false;
    std::memcpy(out, uncommitted_, len);
    return true;
}

Status EmergencyJournal::eraseAndCheck(uint32_t sector)
{
    inc(stats_.erases);
    const uint32_t base = slotAddress(sector, 0u);
    Status s = flash_.eraseSector(base);
    if (isOk(s)) {
        for (uint32_t i = 0; i < slotsPerSector_ && isOk(s); ++i) {
            s = flash_.read(slotAddress(sector, i), slot_, sizeof(slot_));
            if (isOk(s) && !erased(slot_, sizeof(slot_))) s = Status::VerifyFailed;
        }
    }
    if (!isOk(s)) inc(stats_.eraseFailures);
    return s;
}

Status EmergencyJournal::prepareSlot()
{
    if (cursorSlot_ < slotsPerSector_) return Status::Ok;
    const uint32_t next = (cursorSector_ + 1u) % region_.sectors;
    if (next == latestSector_) return Status::InvalidArgument;   // never erase the only valid copy (cannot happen)
    if (!nextErased_) {
        const Status s = eraseAndCheck(next);
        if (!isOk(s)) return s;
    }
    cursorSector_ = next;
    cursorSlot_ = 0u;
    nextErased_ = false;   // the sector after this one holds older records
    return Status::Ok;
}

bool EmergencyJournal::maintenanceDue() const
{
    if (mount_ == JournalMount::NotMounted || mount_ == JournalMount::Unavailable || nextErased_) return false;
    const uint32_t next = (cursorSector_ + 1u) % region_.sectors;
    return cursorSlot_ >= slotsPerSector_ / 2u && next != latestSector_;
}

Status EmergencyJournal::maintain()
{
    if (!maintenanceDue()) return Status::Ok;
    const Status s = eraseAndCheck((cursorSector_ + 1u) % region_.sectors);
    if (isOk(s)) nextErased_ = true;
    return s;
}

Status EmergencyJournal::commit(const uint8_t* payload, size_t len, uint32_t uptimeMs)
{
    if (payload == nullptr || len != journal::kPayloadBytes) return Status::InvalidArgument;
    if (mount_ == JournalMount::NotMounted || mount_ == JournalMount::Unavailable) return Status::NotInitialized;
    if (seq_ >= journal::kMaxSequence) {   // generation exhausted: refuse rather than wrap (reported)
        inc(stats_.commitFailures);
        return Status::OutOfRange;
    }
    Status s = prepareSlot();
    if (!isOk(s)) {
        inc(stats_.commitFailures);
        return s;
    }
    const uint32_t seq = ++seq_;   // consumed even if this slot fails: retries never reuse it
    const uint32_t addr = slotAddress(cursorSector_, cursorSlot_);
    encodeSlot(slot_, seq, boot_, payload, uptimeMs);
    uint8_t check[journal::kSlotBytes];

    // 1) body, 2) read back, 3) commit word, 4) read back.
    s = flash_.program(addr, slot_, journal::kBodyBytes);
    if (isOk(s)) s = flash_.read(addr, check, journal::kBodyBytes);
    if (isOk(s) && std::memcmp(check, slot_, journal::kBodyBytes) != 0) {
        inc(stats_.verifyFailures);
        s = Status::VerifyFailed;
    }
    if (isOk(s)) s = flash_.program(addr + journal::kCommitOffset, &slot_[journal::kCommitOffset], 4u);
    if (isOk(s)) s = flash_.read(addr + journal::kCommitOffset, check, 4u);
    if (isOk(s) && std::memcmp(check, &slot_[journal::kCommitOffset], 4u) != 0) {
        inc(stats_.verifyFailures);
        s = Status::VerifyFailed;
    }
    ++cursorSlot_;   // success or not, this slot is used (a failed one is "burned")
    if (!isOk(s)) {
        inc(stats_.burnedSlots);
        inc(stats_.commitFailures);
        return s;
    }
    std::memcpy(latest_, payload, sizeof(latest_));
    haveLatest_ = true;
    haveUncommitted_ = false;
    latestSector_ = cursorSector_;
    if (mount_ != JournalMount::Recovered) mount_ = JournalMount::Recovered;
    inc(stats_.commits);
    return Status::Ok;
}

}  // namespace pg
