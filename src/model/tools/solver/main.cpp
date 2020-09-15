// #define DISCONTINIOUS
#include "src/Model.cpp"




int main(int argc, char** argv)
{
	//generate model-class


	//Model mdl(argv[1]);
	Model mdl("config.txt");

	cout << "q  = [" << mdl.q.transpose() << "]\n" << endl;
	cout << "qd = [" << mdl.qd.transpose() << "]\n" << endl;
	cout << "Sa = [" << mdl.Sa << "]\n" << endl;
	cout << "NState = [" << mdl.NState << "]\n" << endl;
	cout << "K1 = [" << mdl.K1 << "]\n" << endl;
	cout << "K2 = [" << mdl.K2 << "]\n" << endl;
	cout << "Sc = [" << mdl.Sc << "]\n" << endl;
	cout << "Bc = [" << mdl.Bc << "]\n" << endl;
	cout << "Ba = [" << mdl.Ba << "]\n" << endl;
	cout << "Ndof = [" << mdl.NDof << "]\n" << endl;
	cout << "G11 = [" << mdl.G11 << "]\n" << endl;
	cout << "G22 = [" << mdl.G22 << "]\n" << endl;
	cout << "G33 = [" << mdl.G33 << "]\n" << endl;
	cout << "Gee = [" << mdl.Gee << "]\n" << endl;


	// // solve system
	mdl.implicit_simulate();
		
	return 0;
}



// REFERENCES
// [1] - https://github.com/stulp/tutorials/blob/master/test.md
