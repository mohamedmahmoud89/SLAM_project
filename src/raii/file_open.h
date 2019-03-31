#include<fstream>
using namespace std;
class file_open{
	ifstream* ifs{nullptr};
	ofstream* ofs{nullptr};
public:
	file_open()=delete;
	explicit file_open(ifstream* in_f,const string& fn):
		ifs(in_f),ofs(nullptr)
	{
		ifs->open(fn,fstream::in);	
	}
	explicit file_open(ofstream* in_f,const string& fn):
		ifs(nullptr),ofs(in_f)
	{
		ofs->open(fn,fstream::out);
	}
	file_open(const file_open& f)=delete;
	file_open& operator=(const file_open& f)=delete;
	~file_open(){
		if(ifs&&ifs->is_open())
			ifs->close();
		if(ofs&&ofs->is_open())
			ofs->close();
	}
};
