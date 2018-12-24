#include <Python.h>
#include <iostream>
using namespace std;

class RndWalkPnt{
	PyObject *pName, *pModule, *pDict, *pFunc;
	PyObject *pArgs, *pValue;
	//int i;
	public:
		int setup(int, char**);
		//RndWalkPnt(int, char**);
		//int nextPlease(void);
		long nextPlease(void);
		int dismiss(void);
};
