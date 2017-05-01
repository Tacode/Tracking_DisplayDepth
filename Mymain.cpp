#include <iostream>
#include "BodyIndex.h"
using namespace std;
int main()
{
	BodyIndexBasic myKinect;
	
	HRESULT hr = myKinect.InitializeDefaultSensor();
	if (SUCCEEDED(hr)){
		while (1){
			myKinect.UpDate();
		}
	}
	else{
		cout << "Kinect Initializetion failed!" << endl;
		system("pause");
	}
	return 0;
}