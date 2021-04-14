#include <iostream>
#include "../include/utilities.hpp"
#include "../include/ode4.hpp"
#include "../include/eom1.hpp"
using namespace Eigen;
using namespace std;

int main() {
    
    double x=1.0;
    double h=0.1;
    while (x<100) {
        x=ode4(&eom1, x, h);
        cout<<x<<endl;
    }
    
}

