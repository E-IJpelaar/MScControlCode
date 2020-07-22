#include "src/Model.cpp"

int main(int argc, char** argv)
{
	//generate model-class
	Model mdl(argv[1]);

	cout << "q = [" << mdl.q.transpose() << "]\n" << endl;
	cout << "qd = [" << mdl.qd.transpose() << "]\n" << endl;
	cout << "Sa = [" << mdl.Sa << "]\n" << endl ;

	// // solve system
	mdl.implicit_simulate();
		
	return 0;
}

// REFERENCES
// [1] - https://github.com/stulp/tutorials/blob/master/test.md
