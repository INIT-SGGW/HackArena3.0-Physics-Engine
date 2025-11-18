#include <iostream>
#include <cstdlib>

int main() {
    std::cout << "ASAN Test Program\n";

    // 1. Buffer overflow
    int* arr = new int[5];
    arr[5] = 10; // Out-of-bounds write (valid indices: 0-4)

    // 2. Use-after-free
    int* ptr = new int(42);
    delete ptr;
    std::cout << *ptr << "\n"; // Use-after-free

    // 3. Memory leak
    int* leak = new int[10]; // Never deleted

    return 0;
}
