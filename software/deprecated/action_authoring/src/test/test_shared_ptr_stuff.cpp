//#include "SharedPtrHash.h"

#include <boost/shared_ptr.hpp>
#include <boost/unordered_map.hpp>
#include <map>
#include <string>
#include <iostream>

using namespace std;
using namespace boost;

int main()
{

    int *a = new int[5];
    a[2] = 7;

    int *b = new int[9];
    b[2] = -1;

    shared_ptr<int> aptr(a);
    //shared_ptr<int> aptrCopy(aptr);
    shared_ptr<int> aptrCopy = aptr;


    shared_ptr<int> bptr(b);
    //shared_ptr<int> bptrCopy(bptr);
    shared_ptr<int> bptrCopy = bptr;




    std::map<shared_ptr<int>, string> m;    //comment this out and uncomment the next line to try unordered_map
    //boost::unordered_map<shared_ptr<int>, string> m;

    m[aptr] = "a";
    m[bptr] = "b";

    if (m.find(aptrCopy) == m.end())
    {
        cout << "\n shared_ptr copy not recognized as being in map" << endl;
    }

    cout << "\n using copy we found m[aCopy] = " << m[aptrCopy] << endl;
    cout << "\n using copy we found m[bCopy] = " << m[bptrCopy] << endl;

    cout << "\n comparing copies: " << (aptr == aptrCopy)
         << "\t:" << (bptr == bptrCopy) << endl;

    return 0;
}
