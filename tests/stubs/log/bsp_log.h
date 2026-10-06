#pragma once
void TestLog(const char*, ...);
#define LOGERROR(...) TestLog(__VA_ARGS__)
#define LOGINFO(...) TestLog(__VA_ARGS__)
