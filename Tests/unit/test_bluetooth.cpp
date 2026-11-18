/**
 * EZ-Serial link (framing only) and SOS formatter tests.
 * The lines used here are arbitrary test strings in the "@R,..."/"@E,..."
 * shape; they are NOT claimed to be real CYW20822 commands or events.
 */
#include <cstring>
#include <string>

#include "TestFramework.hpp"
#include "mocks/FakeUart.hpp"

#include "Communications/Bluetooth/EzSerialLink.hpp"
#include "Communications/Bluetooth/SosMessage.hpp"

using namespace pg;

namespace {
struct Recorder final : public IEzSerialListener {
    int responses = 0;
    int events = 0;
    int other = 0;
    std::string lastFirstField;
    size_t lastFieldCount = 0;
    void onPacket(const ezs::Packet& p) override
    {
        if (p.kind == ezs::PacketKind::Response) ++responses;
        if (p.kind == ezs::PacketKind::Event) ++events;
        if (p.kind == ezs::PacketKind::Other) ++other;
        lastFieldCount = p.fieldCount;
        lastFirstField = p.fieldCount ? p.fields[0] : "";
    }
};

size_t feed(EzSerialLink& link, const char* text, uint32_t now = 0)
{
    return link.processBytes(reinterpret_cast<const uint8_t*>(text), std::strlen(text), now);
}
}  // namespace

TEST(ezs_command_framing_appends_crlf)
{
    FakeByteWriter tx;
    EzSerialLink link(tx);
    CHECK(isOk(link.sendCommand("TEST CMD", 0)));
    CHECK(tx.written == "TEST CMD\r\n");
    CHECK(link.awaitingResponse());
    CHECK(link.sendCommand("SECOND", 1) == Status::Busy);  // one outstanding command
}

TEST(ezs_rejects_bad_commands)
{
    FakeByteWriter tx;
    EzSerialLink link(tx);
    CHECK(link.sendCommand("", 0) == Status::InvalidArgument);
    CHECK(link.sendCommand(nullptr, 0) == Status::InvalidArgument);
    CHECK(link.sendCommand("A\r\nB", 0) == Status::InvalidArgument);  // embedded CRLF
    std::string longCmd(200, 'X');
    CHECK(link.sendCommand(longCmd.c_str(), 0) == Status::InvalidArgument);
    CHECK(tx.written.empty());
    tx.result = Status::Timeout;
    CHECK(link.sendCommand("OK", 0) == Status::Timeout);
    CHECK(!link.awaitingResponse());
}

TEST(ezs_response_and_event_classification)
{
    FakeByteWriter tx;
    EzSerialLink link(tx);
    Recorder rec;
    link.setListener(&rec);
    link.sendCommand("X", 0);
    CHECK_EQ(feed(link, "@E,0010,AAA,1\r\n@R,0008,BBB,0000,x=1\r\nnoise line\r\n"), 3u);
    CHECK_EQ(rec.events, 1);
    CHECK_EQ(rec.responses, 1);
    CHECK_EQ(rec.other, 1);
    CHECK(!link.awaitingResponse());
    CHECK(link.state() == BleLinkState::ModuleActive);
}

TEST(ezs_fields_split)
{
    char line[] = "@R,000C,NAME,0000,a=1,b=2";
    ezs::Packet p{};
    ezs::parseLine(line, p);
    CHECK(p.kind == ezs::PacketKind::Response);
    CHECK_EQ(p.fieldCount, 5u);
    CHECK(std::strcmp(p.fields[0], "000C") == 0);
    CHECK(std::strcmp(p.fields[1], "NAME") == 0);
    CHECK(std::strcmp(p.fields[4], "b=2") == 0);
}

TEST(ezs_line_split_across_reads)
{
    FakeByteWriter tx;
    EzSerialLink link(tx);
    Recorder rec;
    link.setListener(&rec);
    CHECK_EQ(feed(link, "@E,00"), 0u);
    CHECK_EQ(feed(link, "05,ZZ"), 0u);
    CHECK_EQ(feed(link, "\r\n"), 1u);
    CHECK_EQ(rec.events, 1);
    CHECK(rec.lastFirstField == "0005");
}

TEST(ezs_overlong_line_discarded_safely)
{
    FakeByteWriter tx;
    EzSerialLink link(tx);
    Recorder rec;
    link.setListener(&rec);
    std::string big(500, 'A');
    big += "\r\n@E,0001,OK\r\n";
    CHECK_EQ(feed(link, big.c_str()), 1u);  // only the good line delivered
    CHECK_EQ(link.stats().overlongLines, 1u);
    CHECK_EQ(rec.events, 1);
}

TEST(ezs_response_timeout_and_fault)
{
    FakeByteWriter tx;
    EzSerialLink link(tx, 1000);
    for (uint32_t i = 0; i < 3; ++i) {
        const uint32_t t0 = i * 5000;
        CHECK(isOk(link.sendCommand("PING?", t0)));
        link.poll(t0 + 999);
        CHECK(link.awaitingResponse());
        link.poll(t0 + 1000);
        CHECK(!link.awaitingResponse());
    }
    CHECK_EQ(link.stats().responseTimeouts, 3u);
    CHECK(link.state() == BleLinkState::Fault);
    feed(link, "@E,0001,X\r\n");  // module talks again
    CHECK(link.state() == BleLinkState::ModuleActive);
}

TEST(sos_record_format)
{
    SosPayload p{};
    p.reason = SosReason::ImpactDetected;  // 2
    p.positionValid = true;
    p.latitude_1e7deg = 510447331;
    p.longitude_1e7deg = -1140718831;
    p.altitudeMsl_dm = 10451;
    p.batteryMillivolts = 3700;
    p.uptimeSeconds = 123;
    char buf[96];
    const size_t n = formatSosRecord(p, buf, sizeof(buf));
    CHECK(std::string(buf) == "SOS,2,1,510447331,-1140718831,10451,3700,123");
    CHECK_EQ(n, std::strlen(buf));
}

TEST(sos_record_extreme_values_and_small_buffer)
{
    SosPayload p{};
    p.latitude_1e7deg = INT32_MIN;
    char buf[96];
    CHECK(formatSosRecord(p, buf, sizeof(buf)) > 0u);
    CHECK(std::strstr(buf, "-2147483648") != nullptr);
    char tiny[10];
    CHECK_EQ(formatSosRecord(p, tiny, sizeof(tiny)), 0u);  // reports truncation
    CHECK_EQ(tiny[9], '\0');                               // still terminated, no overflow
}
