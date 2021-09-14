#include <string>
#include <vector>
#include <iostream>
using namespace std;
int main() {
    vector<int> numbers = {1, 2, 3};
    vector<string> names = {"Igor", "Cyrill"};
    names.push_back("another_string");
    cout << "First name: " << names.back() << endl;
    cout << "Last number: " << numbers.back() << endl; return 0;
}