// Includes the installed umbrella header (it pulls in every other header, so a header missing from the install
// fails here) and links the installed static core.
#include <bclibc.hpp>
#include <cstdio>

int main()
{
    bclibc::BCLIBC_BaseEngine engine;
    (void)engine;
    std::puts("bclibc_core ok");
    return 0;
}
