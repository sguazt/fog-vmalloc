#include "TrivediPerc.hpp"

int TrivediPerc::setup(int argc, char *argv[]){
	// Initialize the Python Interpreter
    	Py_Initialize();
	PySys_SetArgv(argc, argv);

    	// Build the name object
    	pName = PyString_FromString(argv[1]);
    	// Error checking of pName left out 

    	// Load the module object
    	pModule = PyImport_ImportModule(argv[1]);

    	// Clean up
    	Py_DECREF(pName);

    	if(pModule == NULL){
		PyErr_Print();
        	fprintf(stderr, "Failed to load \"%s\"\n", argv[1]);
        	return -1;
	}

	//Get function name
	pFunc = PyObject_GetAttrString(pModule, argv[2]);

	if (pFunc && PyCallable_Check(pFunc))
		return 1;

	if (PyErr_Occurred()){
               	PyErr_Print();
            	fprintf(stderr, "Cannot find function \"%s\"\n", argv[2]);
		return -1;
	}

	return 1;
}


long TrivediPerc::nextPlease(int argc, char *argv[]){
	pArgs = PyTuple_New(argc - 3);
       	for (int i = 0; i < argc - 3; ++i) {
               	//pValue = PyInt_FromLong(atoi(argv[i + 3]));
               	pValue = PyFloat_FromDouble(atof(argv[i + 3]));
		if (!pValue) {
              		Py_DECREF(pArgs);
               		Py_DECREF(pModule);
               		fprintf(stderr, "Cannot convert argument\n");
               		return -1;
		}
			PyTuple_SetItem(pArgs, i, pValue);
        }

	pValue = PyObject_CallObject(pFunc, pArgs);
        //Py_DECREF(pArgs);
        if (pValue != NULL) {
       		//printf("Result of call: %ld\n", PyInt_AsLong(pValue));
        	//Py_DECREF(pValue);
		//return PyInt_AsLong(pValue);
		auto ret = PyInt_AsLong(pValue);
		Py_CLEAR(pValue);
       		return ret;
        } else {
        	//Py_DECREF(pFunc);
        	//Py_DECREF(pModule);
        	PyErr_Print();
        	fprintf(stderr,"Call failed\n");
        	return -1;
        }
}

int TrivediPerc::dismiss(void){
	Py_XDECREF(pFunc);
        Py_DECREF(pModule);
    	Py_Finalize();
	return 1;
}
