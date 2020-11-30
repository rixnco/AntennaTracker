#include <unity.h>
#include "test_main.h"

int main(int argc, char **argv) {
    UNITY_BEGIN();

    RUN_TEST(test_SPortDecoder);
    RUN_TEST(test_CRSFDecoder);

    UNITY_END();
}

