#include <iostream>
#include <thread>

class A {
public:
    void foo(int* n ) { std::cout << *n << std::endl; }
};

int main()
{
    A a;
	int* i = new int(100);
	
    std::thread t1(&A::foo, &a, i);

    t1.join();
    return 0;
}
