#include <iostream>
#include <leg-odometry/kalman_filter.hpp>
#include <unistd.h>
#include <tr1/functional>
#include <leg-odometry/KalmanFilter_Types.hpp>

using namespace std;


//-----------------------------------------------------------------------------------------
   // 3.5 Example A: Callback to member function using an additional argument
   // Task: The function 'DoItA' makes something which implies a callback to
   //       the member function 'Display'. Therefore the wrapper-function
   //       'Wrapper_To_Call_Display is used.



   class TClassA
   {
   public:

      void Display(const int text);
      static void Wrapper_To_Call_Display(void* pt2Object, int text);

      /* more of TClassA */
   };
   
   
   void TClassA::Display(const int text) {
	   cout << text << endl;
   }


   // static wrapper-function to be able to callback the member function Display()
   void TClassA::Wrapper_To_Call_Display(void* pt2Object, int string)
   {
       // explicitly cast to a pointer to TClassA
       TClassA* mySelf = (TClassA*) pt2Object;

       // call member
       mySelf->Display(string);
   }


   // function does something which implies a callback
   // note: of course this function can also be a member function
   void DoItA(void* pt2Object, void (*pt2Function)(void* pt2Object, int text))
   {
      /* do something */

      pt2Function(pt2Object, 2);  // make callback
   }


   // execute example code
   void Callback_Using_Argument()
   {
      // 1. instantiate object of TClassA
      TClassA objA;

      // 2. call 'DoItA' for <objA>
      DoItA((void*) &objA, TClassA::Wrapper_To_Call_Display);
   }


   
   
   
       class cCallback
       {
	    public:
	    	virtual void Execute(int Param) const =0;
       };
	     
	     
	    template <class cInstance>
	    class TCallback : public cCallback
	    {
	    public:
	    	TCallback() // constructor
		    {
	    		pFunction = 0;
		    }
	     
		    typedef void (cInstance::*tFunction)(int Param);
		    
		    virtual void Execute(int Param) const
		    {
			    if (pFunction) (cInst->*pFunction)(Param);
			    else printf("ERROR : the callback function has not been defined !!!!");
		    }
		     
		    void SetCallback (cInstance *cInstancePointer, tFunction pFunctionPointer)
		    {
			    cInst = cInstancePointer;
			    pFunction = pFunctionPointer;
		    }
		     
	    private:
		    cInstance *cInst;
		    tFunction pFunction;
	    };
   
	    
	    void TestTheCallback(cCallback *pCallbackFunction, int Param)
	    {
	    	pCallbackFunction->Execute(Param * Param);
	    }
	    
	    
	    
	    /*class KalmanFilter_Model {
	    public:
	    	
	    	void propagation_model(int Param);
	    	void measurement_model(int Param);
	    
	    };
	    
	    
	    void KalmanFilter_Model::propagation_model(int Param) {
	    	cout << "We found this number in KalmanFilter_Model::propagation_model: " << Param << endl;
	    	cout << Param << endl;
	    }
	    
	    void KalmanFilter_Model::measurement_model(int Param) {
	    	cout << "We found this number in KalmanFilter_Model::measurement_model: " << Param << endl;
	    }
	    */
	    
	    class cMyProject {
	    public:
	    	void CallbackFox(int Param);
	    	void CallbackDemo();
	    	
	    protected:
	    	TCallback<cMyProject> i_Callback;
	    	
	    };
	    
	    void cMyProject::CallbackFox(int Param)
	    {
		    //char Buf[50];
		    cout << "Now I'm in Fox with Param " << Param << endl;
		    //printf(Buf);
	    }
	    
	    void cMyProject::CallbackDemo()
	    {
			// defining where the callback should jump to
		    i_Callback.SetCallback(this, &cMyProject::CallbackFox);
		    //i_Callback.SetCallback(this, &cMyProject::propagation_model);
		    		    
		    		    
			// now you can pass i_Callback like a pointer to a function
		    TestTheCallback(&i_Callback, 2);
	    }

	    /*
	    //Declaration:
	    typedef int (ClassName::*CallbackType)(float);
	    
	    //This method performs work using an object pointer
	    void DoWorkPointer(CallbackType callback)
	    {
	      //Class pointer to invoke it through
	      ClassName * pointerInstance;

	      //Invocation
	      int result = (pointerInstance->*callback)(1.0f);
	    }*/
	    
	    
	    /*
	    class GameCharacter;
	    int defaultHealthCalc(const GameCharacter& gc);

	    class GameCharacter
	    {
	    public:
	      typedef std::tr1::function<int (const GameCharacter&)> HealthCalcFunc;

	      explicit GameCharacter(HealthCalcFunc hcf = defaultHealthCalc) : healthFunc(hcf)
	      { }

	      int healthValue() const { return healthFunc(*this); }

	    private:
	      HealthCalcFunc healthFunc;
	    };*/
	    
	    

int main() {
	
	cout << "This is a test application to stimulate and test some of the kalman_filter class functionality." << endl << endl;
	
	cout << "Starting tests..." << endl << endl;
	
	KalmanFilter kf;
	
	kf.Initialize();
	
	// Something like here is the model
	//kf.setModel(*prop, *meas); // for numerical derived jacobian
	//kf.setModel(*prop, *meas, *trans_Jacobian, *meas_Jacobian); // This will use analytical derivatives
	
	// then we need data
	
	// we iterate through all the events
	// publish the output from this process
	
	//cout << "trying the callback function" << endl;
	//Callback_Using_Argument();
	
	cMyProject cb_test;
	//cb_test.CallbackDemo();
	
	//DoWorkPointer(&cMyProject::CallbackFox);
	
	
	//GameCharacter *gc;
	
	
	cout << "Success" << endl;
	return 0;
}

void testcallback(Eigen::VectorXd state) {
	
	return;
}

