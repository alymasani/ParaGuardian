/**
 ******************************************************************************
 * @file    TestFramework.hpp
 * @brief   Minimal self-registering unit-test framework (host only).
 *
 *   TEST(name) { CHECK(x); CHECK_EQ(a, b); CHECK_NEAR(a, b, tol); }
 *
 * A failing CHECK prints file:line and the expression, marks the test
 * failed, and returns from that test (other tests still run).
 ******************************************************************************
 */
#ifndef PARAGUARDIAN_TESTS_TEST_FRAMEWORK_HPP
#define PARAGUARDIAN_TESTS_TEST_FRAMEWORK_HPP

#include <cmath>
#include <cstdio>
#include <cstdlib>

namespace testfw {

using TestFn = void (*)();

struct TestCase {
    const char* name;
    TestFn fn;
};

inline TestCase* registry() { static TestCase cases[512]; return cases; }
inline int& registryCount() { static int n = 0; return n; }
inline bool& currentFailed() { static bool f = false; return f; }

struct Registrar {
    Registrar(const char* name, TestFn fn)
    {
        if (registryCount() < 512) {
            registry()[registryCount()++] = TestCase{name, fn};
        }
    }
};

inline void fail(const char* file, int line, const char* msg)
{
    std::printf("    FAIL %s:%d: %s\n", file, line, msg);
    currentFailed() = true;
}

inline int runAll()
{
    int failed = 0;
    for (int i = 0; i < registryCount(); ++i) {
        currentFailed() = false;
        registry()[i].fn();
        std::printf("[%s] %s\n", currentFailed() ? "FAIL" : " OK ", registry()[i].name);
        if (currentFailed()) {
            ++failed;
        }
    }
    std::printf("\n%d tests, %d passed, %d failed\n", registryCount(), registryCount() - failed, failed);
    return failed == 0 ? 0 : 1;
}

}  // namespace testfw

#define TEST(name)                                                    \
    static void name();                                               \
    static testfw::Registrar registrar_##name(#name, &name);          \
    static void name()

#define CHECK(cond)                                                   \
    do {                                                              \
        if (!(cond)) {                                                \
            testfw::fail(__FILE__, __LINE__, #cond);                  \
            return;                                                   \
        }                                                             \
    } while (0)

#define CHECK_EQ(a, b)                                                                              \
    do {                                                                                            \
        const auto va_ = (a);                                                                       \
        const auto vb_ = (b);                                                                       \
        if (!(va_ == vb_)) {                                                                        \
            char msg_[256];                                                                         \
            std::snprintf(msg_, sizeof(msg_), "%s == %s  (got %lld vs %lld)", #a, #b,               \
                          static_cast<long long>(va_), static_cast<long long>(vb_));                \
            testfw::fail(__FILE__, __LINE__, msg_);                                                 \
            return;                                                                                 \
        }                                                                                           \
    } while (0)

#define CHECK_NEAR(a, b, tol)                                                                       \
    do {                                                                                            \
        const double va_ = static_cast<double>(a);                                                  \
        const double vb_ = static_cast<double>(b);                                                  \
        if (std::fabs(va_ - vb_) > (tol)) {                                                         \
            char msg_[256];                                                                         \
            std::snprintf(msg_, sizeof(msg_), "%s ~= %s  (got %.9g vs %.9g, tol %g)", #a, #b, va_,  \
                          vb_, static_cast<double>(tol));                                           \
            testfw::fail(__FILE__, __LINE__, msg_);                                                 \
            return;                                                                                 \
        }                                                                                           \
    } while (0)

#endif  // PARAGUARDIAN_TESTS_TEST_FRAMEWORK_HPP
