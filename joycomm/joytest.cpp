
#include <Python.h>

using namespace std;

void exec_interactive_interpreter(int argc, char *argv[]) {
    Py_Initialize();
    Py_Main(argc, argv);
    Py_Finalize();
}

int main(int argc, char* argv[]) {



return 0;

}

