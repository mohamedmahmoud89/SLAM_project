#include"test_odom.h"
#include"test_scan.h"
#include"test_klmn.h"
using namespace std;

int main(){
	// Odom test	
	Test_Odom();

	// Scan test
	Test_Scan();
	
	// KLMN test
	Test_Klmn();

	return 0;
}

