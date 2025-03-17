#include <stdint.h>

int main() {
    volatile int a = 5;
    volatile int b = 3;
    volatile int c = a + b;
    return c;
}