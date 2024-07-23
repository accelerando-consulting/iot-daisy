#define BUILD_NUMBER 11
#define VERSION 1

/*@******************************** Macros **********************************/

#if (SDI_PRINT || DEBUG)
#define LOG(...) printf(__VA_ARGS__)
#else
#define LOG(...) {}
#endif

