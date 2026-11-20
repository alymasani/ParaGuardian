/**
 ******************************************************************************
 * @file    test_storage_journal.cpp
 * @brief   Milestone 8: emergency journal power-loss safety, flight / critical
 *          log recovery and rotation, storage scheduling priority.
 *          All against FakeNorFlash (NOR semantics + fault injection).
 ******************************************************************************
 */
#include <cstdio>
#include <cstring>
#include <vector>

#include "TestFramework.hpp"
#include "mocks/FakeNorFlash.hpp"
#include "mocks/MockEmergencyIo.hpp"
#include "Storage/EmergencyJournal.hpp"
#include "Storage/FlightLog.hpp"
#include "Storage/FlightRecorder.hpp"
#include "Storage/JournaledEmergencyStore.hpp"
#include "Storage/StorageService.hpp"

using namespace pg;

namespace {
const FlashRegion kJ = partition::kEmergencyJournal;

void payload(uint8_t* p, uint32_t v)
{
    for (size_t i = 0; i < journal::kPayloadBytes; ++i) p[i] = static_cast<uint8_t>(v * 7u + i);
}
bool payloadIs(const uint8_t* p, uint32_t v)
{
    uint8_t q[journal::kPayloadBytes];
    payload(q, v);
    return std::memcmp(p, q, sizeof(q)) == 0;
}

LogEntry entry(LogType t, uint32_t ts, uint8_t fill, uint8_t len = 20u)
{
    LogEntry e{};
    e.type = t;
    e.length = len;
    e.timestampMs = ts;
    for (uint8_t i = 0; i < len; ++i) e.payload[i] = static_cast<uint8_t>(fill + i);
    return e;
}

struct Collected {
    std::vector<uint32_t> seqs;
    std::vector<uint32_t> stamps;
};
void collect(void* ctx, const LogEntry& e, uint32_t seq)
{
    auto* c = static_cast<Collected*>(ctx);
    c->seqs.push_back(seq);
    c->stamps.push_back(e.timestampMs);
}
}  // namespace

// ---------------------------------------------------------------------------
TEST(journal_blank_commit_remount_and_boot_count)
{
    FakeNorFlash f;
    EmergencyJournal j(f, kJ);
    CHECK(j.mount() == JournalMount::Blank);
    uint8_t p[32], out[32];
    payload(p, 1u);
    CHECK(isOk(j.commit(p, 32, 10u)));
    CHECK(j.latest(out, 32) && payloadIs(out, 1u));
    EmergencyJournal j2(f, kJ);   // reset
    CHECK(j2.mount() == JournalMount::Recovered);
    CHECK(j2.latest(out, 32) && payloadIs(out, 1u));
    CHECK_EQ(j2.bootCount(), 2u);
    CHECK_EQ(j2.sequence(), 1u);
    payload(p, 2u);
    CHECK(isOk(j2.commit(p, 32, 20u)));
    EmergencyJournal j3(f, kJ);
    CHECK(j3.mount() == JournalMount::Recovered);
    CHECK(j3.latest(out, 32) && payloadIs(out, 2u));
    CHECK_EQ(j3.bootCount(), 3u);
}

TEST(journal_rotation_keeps_latest_and_bounds_wear)
{
    FakeNorFlash f;
    EmergencyJournal j(f, kJ);
    j.mount();
    uint8_t p[32], out[32];
    for (uint32_t i = 1; i <= 300u; ++i) {
        payload(p, i % 30u);
        CHECK(isOk(j.commit(p, 32, i)));
        if (i % 37u == 0u) {   // reset now and then: always the newest record
            EmergencyJournal r(f, kJ);
            CHECK(r.mount() == JournalMount::Recovered);
            CHECK(r.latest(out, 32) && payloadIs(out, i % 30u));
        }
    }
    const uint32_t s0 = kJ.base / w25q::SECTOR_SIZE;
    std::printf("    [storage] 300 commits: sector erases %u + %u (64 slots per sector)\n",
                static_cast<unsigned>(f.eraseCount[s0]), static_cast<unsigned>(f.eraseCount[s0 + 1]));
    CHECK(f.eraseCount[s0] + f.eraseCount[s0 + 1] <= 300u / 64u + 1u);
}

TEST(journal_power_cut_at_every_byte_never_yields_garbage)
{
    // Commit record 5, then cut power at every possible byte of the NEXT commit (record 6).
    unsigned keptOld = 0, gotNew = 0, uncommitted = 0;
    for (long cut = 0; cut <= 64; ++cut) {
        FakeNorFlash f;
        f.seed = 1000u + static_cast<uint32_t>(cut);
        EmergencyJournal j(f, kJ);
        j.mount();
        uint8_t p[32], out[32];
        for (uint32_t v = 1; v <= 5u; ++v) {
            payload(p, v);
            CHECK(isOk(j.commit(p, 32, v)));
        }
        f.cutAfterBytes = cut;
        payload(p, 6u);
        const Status s = j.commit(p, 32, 6u);
        f.reboot();
        EmergencyJournal r(f, kJ);
        CHECK(r.mount() == JournalMount::Recovered);
        CHECK(r.latest(out, 32));
        if (isOk(s)) {
            CHECK(payloadIs(out, 6u));
            ++gotNew;
        } else {
            CHECK(payloadIs(out, 5u));   // the previous committed record, never garbage, never record 6
            ++keptOld;
            if (r.uncommitted(out, 32)) {
                CHECK(payloadIs(out, 6u));   // intact body, missing commit word
                ++uncommitted;
            }
        }
        // the journal keeps working after the cut
        payload(p, 7u);
        CHECK(isOk(r.commit(p, 32, 7u)));
        EmergencyJournal r2(f, kJ);
        r2.mount();
        CHECK(r2.latest(out, 32) && payloadIs(out, 7u));
    }
    std::printf("    [storage] 65 cut points: %u kept previous record, %u committed, %u intact-uncommitted seen\n",
                keptOld, gotNew, uncommitted);
    CHECK(keptOld >= 60u);
}

TEST(journal_power_cut_during_rotation_erase_keeps_latest)
{
    FakeNorFlash f;
    EmergencyJournal j(f, kJ);
    j.mount();
    uint8_t p[32], out[32];
    for (uint32_t i = 1; i <= 64u; ++i) {   // fill sector A exactly
        payload(p, i % 30u);
        CHECK(isOk(j.commit(p, 32, i)));
    }
    // fill sector B, then rotate back to A: A holds older records only
    for (uint32_t i = 65; i <= 128u; ++i) {
        payload(p, i % 30u);
        CHECK(isOk(j.commit(p, 32, i)));
    }
    f.cutNextErase = true;   // the erase of sector A for record 129 is interrupted
    payload(p, 129u % 30u);
    CHECK(!isOk(j.commit(p, 32, 129u)));
    f.reboot();
    EmergencyJournal r(f, kJ);
    CHECK(r.mount() == JournalMount::Recovered);
    CHECK(r.latest(out, 32) && payloadIs(out, 128u % 30u));
    payload(p, 3u);
    CHECK(isOk(r.commit(p, 32, 130u)));   // erases A again, writes there
    EmergencyJournal r2(f, kJ);
    r2.mount();
    CHECK(r2.latest(out, 32) && payloadIs(out, 3u));
}

TEST(journal_crc_corruption_falls_back_and_all_corrupt_is_reported)
{
    FakeNorFlash f;
    EmergencyJournal j(f, kJ);
    j.mount();
    uint8_t p[32], out[32];
    payload(p, 1u);
    j.commit(p, 32, 1u);
    payload(p, 2u);
    j.commit(p, 32, 2u);
    f.flip(kJ.base + 64u + 20u, 0x01u);   // bit rot in the newest slot's payload
    EmergencyJournal r(f, kJ);
    CHECK(r.mount() == JournalMount::Recovered);
    CHECK(r.latest(out, 32) && payloadIs(out, 1u));
    CHECK_EQ(r.stats().tornAtMount, 1u);
    f.flip(kJ.base + 20u, 0x01u);         // and the older one
    EmergencyJournal r2(f, kJ);
    CHECK(r2.mount() == JournalMount::Corrupt);
    CHECK(!r2.latest(out, 32));
    payload(p, 9u);
    CHECK(isOk(r2.commit(p, 32, 3u)));    // usable again, sequence above every intact slot
    EmergencyJournal r3(f, kJ);
    CHECK(r3.mount() == JournalMount::Recovered);
    CHECK(r3.latest(out, 32) && payloadIs(out, 9u));
}

TEST(journal_program_failure_burns_slot_and_retry_succeeds)
{
    FakeNorFlash f;
    EmergencyJournal j(f, kJ);
    j.mount();
    uint8_t p[32], out[32];
    payload(p, 1u);
    f.failPrograms = 1;
    CHECK(!isOk(j.commit(p, 32, 1u)));
    CHECK(isOk(j.commit(p, 32, 2u)));
    CHECK_EQ(j.stats().burnedSlots, 1u);
    EmergencyJournal r(f, kJ);
    r.mount();
    CHECK(r.latest(out, 32) && payloadIs(out, 1u));
    CHECK_EQ(r.sequence(), 2u);   // the burned slot's sequence was never reused
}

TEST(journal_unavailable_flash_and_sequence_exhaustion)
{
    FakeNorFlash f;
    f.present = false;
    EmergencyJournal j(f, kJ);
    CHECK(j.mount() == JournalMount::Unavailable);
    uint8_t p[32];
    payload(p, 1u);
    CHECK(j.commit(p, 32, 1u) == Status::NotInitialized);

    // a slot near the end of the sequence space: the journal refuses instead of wrapping
    FakeNorFlash g;
    uint8_t slot[64];
    EmergencyJournal::encodeSlot(slot, journal::kMaxSequence, 7u, p, 0u);
    std::memcpy(&g.mem[kJ.base], slot, sizeof(slot));
    EmergencyJournal k(g, kJ);
    CHECK(k.mount() == JournalMount::Recovered);
    CHECK(k.commit(p, 32, 1u) == Status::OutOfRange);
}

TEST(journal_maintenance_pre_erases_so_commit_does_not_pay_for_it)
{
    FakeNorFlash f;
    EmergencyJournal j(f, kJ);
    j.mount();
    uint8_t p[32];
    payload(p, 1u);
    for (int i = 0; i < 40; ++i) j.commit(p, 32, 0u);
    CHECK(!j.maintenanceDue());                       // blank journal: the other sector is already erased
    for (int i = 0; i < 24 + 40; ++i) j.commit(p, 32, 0u);   // rotate into sector B; A now holds old records
    CHECK_EQ(j.activeSector(), 1u);
    CHECK(j.maintenanceDue());
    CHECK(isOk(j.maintain()));
    CHECK(!j.maintenanceDue());
    for (int i = 0; i < 24; ++i) j.commit(p, 32, 0u);   // sector full
    const uint32_t erasesBefore = f.erases;
    const uint32_t t0 = f.nowMs;
    CHECK(isOk(j.commit(p, 32, 0u)));                 // rotates without an erase
    CHECK_EQ(f.erases, erasesBefore);
    CHECK(f.nowMs - t0 <= 3u);
}

// ---------------------------------------------------------------------------
TEST(log_append_flush_scan_and_resume_sequence_after_reset)
{
    FakeNorFlash f;
    const FlashRegion r{partition::kFlightLog.base, 4u};
    FlightLog log(f, r, logfmt::kTagFlight, LogFullPolicy::OverwriteOldest);
    CHECK(log.mount() == LogMount::Blank);
    for (uint32_t i = 0; i < 50u; ++i) CHECK(isOk(log.append(entry(LogType::Navigation, 1000u + i, 3u))));
    CHECK(isOk(log.flush()));
    FlightLog again(f, r, logfmt::kTagFlight, LogFullPolicy::OverwriteOldest);
    CHECK(again.mount() == LogMount::Recovered);
    CHECK_EQ(again.nextSequence(), 51u);
    CHECK_EQ(again.stats().recoveredRecords, 50u);
    for (uint32_t i = 0; i < 5u; ++i) CHECK(isOk(again.append(entry(LogType::Health, 5000u + i, 9u))));
    again.flush();
    Collected c;
    CHECK_EQ(FlightLog::scan(f, r, logfmt::kTagFlight, collect, &c), 55u);
    for (size_t i = 0; i < c.seqs.size(); ++i) CHECK_EQ(c.seqs[i], static_cast<uint32_t>(i + 1u));
}

TEST(log_power_cut_loses_only_the_record_being_written)
{
    FakeNorFlash f;
    const FlashRegion r{partition::kFlightLog.base, 4u};
    FlightLog log(f, r, logfmt::kTagFlight, LogFullPolicy::OverwriteOldest);
    log.mount();
    for (uint32_t i = 0; i < 20u; ++i) log.append(entry(LogType::Navigation, i, 1u));
    log.flush();                                      // 20 committed
    for (uint32_t i = 20; i < 23u; ++i) log.append(entry(LogType::Navigation, i, 1u));
    f.cutAfterBytes = 36 + 17;                        // cut inside the 2nd of the 3 new records
    CHECK(!isOk(log.flush()));
    f.reboot();
    FlightLog r2(f, r, logfmt::kTagFlight, LogFullPolicy::OverwriteOldest);
    CHECK(r2.mount() == LogMount::Recovered);
    Collected c;
    const size_t n = FlightLog::scan(f, r, logfmt::kTagFlight, collect, &c);
    CHECK_EQ(n, 21u);                                 // 20 + the fully programmed 21st; the torn 22nd is dropped
    CHECK_EQ(c.seqs.back(), 21u);
    // writing continues on a fresh page with a sequence above everything recovered
    CHECK(isOk(r2.append(entry(LogType::Health, 99u, 2u))));
    r2.flush();
    Collected d;
    FlightLog::scan(f, r, logfmt::kTagFlight, collect, &d);
    CHECK_EQ(d.seqs.back(), 22u);
    CHECK_EQ(d.stamps.back(), 99u);
}

TEST(log_rotation_overwrites_oldest_or_stops_when_full)
{
    FakeNorFlash f;
    const FlashRegion r{partition::kFlightLog.base, 3u};
    FlightLog log(f, r, logfmt::kTagFlight, LogFullPolicy::OverwriteOldest);
    log.mount();
    for (uint32_t i = 0; i < 1000u; ++i) CHECK(isOk(log.append(entry(LogType::Navigation, i, 0u, 40u))));
    log.flush();
    Collected c;
    const size_t n = FlightLog::scan(f, r, logfmt::kTagFlight, collect, &c);
    std::printf("    [storage] 1000 x 56-byte records in a 3-sector ring: %zu retained (newest %u), %u rotations\n", n,
                static_cast<unsigned>(c.seqs.back()), static_cast<unsigned>(log.stats().rotations));
    CHECK_EQ(c.seqs.back(), 1000u);
    CHECK(n >= 2u * 64u && n <= 3u * 64u);   // 64 records per sector; one sector was just recycled
    for (size_t i = 1; i < c.seqs.size(); ++i) CHECK_EQ(c.seqs[i], c.seqs[i - 1] + 1u);   // contiguous, ordered

    FakeNorFlash g;
    FlightLog stop(g, r, logfmt::kTagCritical, LogFullPolicy::StopWhenFull);
    stop.mount();
    uint32_t accepted = 0;
    for (uint32_t i = 0; i < 1000u; ++i) accepted += isOk(stop.append(entry(LogType::Emergency, i, 0u, 40u))) ? 1u : 0u;
    stop.flush();
    CHECK(stop.full());
    CHECK(stop.stats().droppedFull > 0u);
    Collected d;
    FlightLog::scan(g, r, logfmt::kTagCritical, collect, &d);
    CHECK_EQ(d.seqs.front(), 1u);   // the oldest records were kept
    CHECK_EQ(static_cast<uint32_t>(d.seqs.size()), accepted);
}

TEST(log_cut_during_rotation_erase_and_corrupt_sector_recovery)
{
    FakeNorFlash f;
    const FlashRegion r{partition::kFlightLog.base, 3u};
    FlightLog log(f, r, logfmt::kTagFlight, LogFullPolicy::OverwriteOldest);
    log.mount();
    uint32_t i = 0;
    while (log.headSector() == 0u) CHECK(isOk(log.append(entry(LogType::Navigation, i++, 0u, 40u))));
    while (log.headSector() == 1u) CHECK(isOk(log.append(entry(LogType::Navigation, i++, 0u, 40u))));
    while (log.headSector() == 2u && log.stats().rotations < 2u) log.append(entry(LogType::Navigation, i++, 0u, 40u));
    log.flush();
    // now force the next rotation (back to sector 0) to be cut mid-erase
    f.cutNextErase = true;
    Status s = Status::Ok;
    for (int k = 0; k < 200 && isOk(s); ++k) s = log.append(entry(LogType::Navigation, i++, 0u, 40u));
    f.reboot();
    FlightLog r2(f, r, logfmt::kTagFlight, LogFullPolicy::OverwriteOldest);
    CHECK(r2.mount() == LogMount::Recovered);
    CHECK_EQ(r2.headSector(), 2u);                     // sector 0 has no valid header any more
    Collected c;
    const size_t n = FlightLog::scan(f, r, logfmt::kTagFlight, collect, &c);
    CHECK(n > 0u);
    for (size_t k = 1; k < c.seqs.size(); ++k) CHECK(c.seqs[k] > c.seqs[k - 1]);
    CHECK(isOk(r2.append(entry(LogType::Health, 7u, 0u))));   // rotates into sector 0 (erased again)
    CHECK(isOk(r2.flush()));
}

// ---------------------------------------------------------------------------
namespace {
struct Rig {
    mock::NoLock lock;
    FakeNorFlash flash;
    JournaledEmergencyStore store{lock};
    LogQueue<16> critical{lock};
    LogQueue<32> telemetry{lock};
    StorageService svc{flash, store, critical, telemetry, StorageServiceConfig{}, &flash};
};
}  // namespace

TEST(storage_service_emergency_commit_preempts_telemetry_flood)
{
    Rig g;
    g.svc.start(0u);
    CHECK(g.store.available());
    CHECK(g.store.mountState() == StoreMount::Blank);
    LogPayload p;
    for (int i = 0; i < 48; ++i) p = LogPayload{}, p.put32(static_cast<uint32_t>(i)), g.telemetry.push(LogType::Navigation, 1u, p);
    CHECK_EQ(g.telemetry.size(), 32u);
    CHECK_EQ(g.telemetry.stats().dropped, 16u);       // overflow counted, never silent
    g.svc.step(1u);                                    // one telemetry unit in progress
    uint8_t rec[kEmergencyRecordBytes];
    payload(rec, 3u);
    uint32_t ticket = 0;
    CHECK(g.store.save(rec, sizeof(rec), ticket) == PersistAck::Queued);
    CHECK(g.store.committedTicket() != ticket);        // queued is NOT saved
    g.svc.step(2u);
    CHECK_EQ(g.store.committedTicket(), ticket);       // the very next unit commits it
    CHECK(g.telemetry.size() > 0u);                    // telemetry still waiting
}

TEST(storage_service_coalesces_and_reports_failed_commits_then_recovers)
{
    Rig g;
    g.svc.start(0u);
    uint8_t rec[kEmergencyRecordBytes];
    uint32_t t1 = 0, t2 = 0;
    payload(rec, 1u);
    g.store.save(rec, sizeof(rec), t1);
    payload(rec, 2u);
    g.store.save(rec, sizeof(rec), t2);                // supersedes t1 before it was written
    g.flash.timeoutWrites = true;
    for (uint32_t t = 10; t < 40u; t += 10) g.svc.step(t);
    CHECK_EQ(g.store.failedTicket(), t2);
    CHECK(g.svc.status().emergencyDegraded);
    g.flash.timeoutWrites = false;
    for (uint32_t t = 40; t < 2000u && g.store.committedTicket() != t2; t += 50) g.svc.step(t);
    CHECK_EQ(g.store.committedTicket(), t2);
    CHECK(!g.svc.status().emergencyDegraded);
    CHECK_EQ(g.store.stats().coalesced, 1u);
    uint8_t out[32];
    EmergencyJournal j(g.flash, kJ);
    j.mount();
    CHECK(j.latest(out, 32) && payloadIs(out, 2u));
}

TEST(storage_service_flash_absent_publishes_unavailable_and_rejects)
{
    Rig g;
    g.flash.present = false;
    g.svc.start(0u);
    CHECK(g.store.mounted());
    CHECK(!g.store.available());
    uint8_t rec[kEmergencyRecordBytes] = {};
    uint32_t t = 7;
    CHECK(g.store.save(rec, sizeof(rec), t) == PersistAck::Rejected);
    CHECK_EQ(t, 0u);
    LogPayload p;
    p.put8(1u);
    g.critical.push(LogType::Boot, 0u, p);
    for (uint32_t k = 0; k < 10u; ++k) g.svc.step(k * 100u);   // never spins, never crashes
    CHECK(g.svc.status().criticalRecords == 0u);
}

TEST(storage_service_logs_and_recorder_round_trip_through_flash)
{
    Rig g;
    g.svc.start(0u);
    FlightRecorder rec(g.critical, g.telemetry, LogProfile::Detailed);
    rec.logBoot(0x1234u, 1u, 2u, 2u, 2u, 0u);
    NavigationEstimate n{};
    n.altitudeValid = n.velocityValid = true;
    n.altitude_m = 123.45f;
    n.verticalVelocity_mps = -1.5f;
    n.altitudeSigma_m = 0.5f;
    for (uint32_t t = 0; t < 2000u; t += 20u) {
        rec.logNavigation(n, t);
        g.svc.step(t);
    }
    EmergencyStatus es{};
    es.state = EmergencyState::Countdown;
    es.incidentId = 4u;
    rec.logEmergency(es, 2000u);
    rec.logEmergency(es, 2020u);   // unchanged: not logged twice
    CHECK(rec.profile() == LogProfile::Emergency);
    for (uint32_t t = 2000u; t < 4000u; t += 20u) g.svc.step(t);
    g.svc.flushAll();
    Collected crit, tele;
    const size_t nc = FlightLog::scan(g.flash, partition::kCriticalLog, logfmt::kTagCritical, collect, &crit);
    const size_t nt = FlightLog::scan(g.flash, partition::kFlightLog, logfmt::kTagFlight, collect, &tele);
    std::printf("    [storage] recorder: %zu critical records, %zu telemetry records (detailed profile, 4 s)\n", nc, nt);
    CHECK_EQ(nc, 2u);                 // boot + one emergency transition
    CHECK(nt >= 18u && nt <= 22u);    // 10 Hz for 2 s
}

TEST(log_record_golden_vector_matches_python_decoder_fixture)
{
    // Tests/vectors/pglog_v1_records.bin is decoded by Tools/pglog/test_pglog_decode.py.
    // This test pins the bytes so firmware and decoder cannot drift apart silently.
    LogPayload p;
    p.put8(2u);            // Navigation: state
    p.putI32(12345);       // alt_cm
    p.putI32(-1500);       // vz_mmps
    p.putI32(INT32_MIN);   // az invalid
    p.putI32(50);          // sigma
    p.put32(0x11u);        // flags
    LogEntry e{};
    e.type = LogType::Navigation;
    e.length = static_cast<uint8_t>(p.size());
    e.timestampMs = 0x01020304u;
    std::memcpy(e.payload, p.data(), p.size());
    uint8_t out[kLogMaxRecord];
    const size_t n = encodeLogRecord(e, 7u, out, sizeof(out));
    CHECK_EQ(n, 37u);
    FILE* fp = std::fopen("vectors/pglog_v1_records.bin", "rb");   // cwd = Tests/ (run_host_tests.sh)
    CHECK(fp != nullptr);
    uint8_t ref[64] = {};
    const size_t got = std::fread(ref, 1, sizeof(ref), fp);
    std::fclose(fp);
    CHECK_EQ(got, n);
    CHECK(std::memcmp(ref, out, n) == 0);
    LogEntry d{};
    uint32_t seq = 0;
    CHECK_EQ(decodeLogRecord(out, n, d, seq), n);
    CHECK_EQ(seq, 7u);
    out[20] ^= 1u;
    CHECK_EQ(decodeLogRecord(out, n, d, seq), 0u);
}
