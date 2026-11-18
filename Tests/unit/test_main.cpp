/**
 * Host test entry point. All TEST() cases self-register from the other files.
 */
#include "TestFramework.hpp"

int main()
{
    return testfw::runAll();
}
