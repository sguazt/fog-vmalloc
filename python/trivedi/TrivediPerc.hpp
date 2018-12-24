#include <Python.h>
#include <iostream>
using namespace std;

class TrivediPerc{
	PyObject *pName, *pModule, *pDict, *pFunc;
	PyObject *pArgs, *pValue;
	//int i;
	public:
		int setup(int, char**);
		//RndWalkPnt(int, char**);
		long nextPlease(int, char**);
		//long nextPlease(void);
		int dismiss(void);
};
