/* 
 * Copyright (C): None
 * Authors: Jimmy Baraglia
 * Public License for more details
*/

#define QUEUE_SIZE 5

#include "Predictor.h"


Predictor::Predictor(ResourceFinder &rf):
maxTests(10000), maxSamples(4), inputNeurons(4),
hiddenNeurons(40), outputNeurons(4), contextNeurons(40),
learnRate(0.05), trainingReps(2000), iterations(0),
testIter(0)
{    
	//Assign paramters values to internal varaibles
	if(rf.find("MAXTESTS").asInt() != 0)
		maxTests = rf.find("MAXTESTS").asInt();// = 10000;
	
	if(rf.find("MAXSAMPLE").asInt() != 0)
		maxSamples = rf.find("MAXSAMPLE").asInt();// = 4;
	
	if(rf.find("INN").asInt() != 0)
		inputNeurons = rf.find("INN").asInt();// = 6;
	if(rf.find("HN").asInt() != 0)
		hiddenNeurons = rf.find("HN").asInt();// = 3;
	if(rf.find("OUTN").asInt() != 0)
		outputNeurons = rf.find("OUTN").asInt();// = 6;
	if(rf.find("CN").asInt() != 0)
		contextNeurons = rf.find("CN").asInt();// = 3;

	if(rf.find("LRATE").asDouble() != 0.0)
		learnRate = rf.find("LRATE").asDouble();// = 0.2;    //Rho.
	if(rf.find("TRAINSTEP").asInt() != 0)
		trainingReps = rf.find("TRAINSTEP").asInt();// = 2000;
	
	//Use files for training
	FILE = rf.find("FILE").asInt();
	
	//Used for the first iteration
	beVector = VectorXd(inputNeurons);//[inputNeurons] = {1.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	//Input buffer
	sampleInput = MatrixXd(2, inputNeurons);
	
	//Initialise all inputs to 0
	for(int i = 0; i <= (inputNeurons-1); i++){
		beVector(i) = 0;
		sampleInput(0, i) = 0;
		sampleInput(1, i) = 0;
	}

	//Input to Hidden weights (with biases).
	wih = MatrixXd(inputNeurons+1, hiddenNeurons);;//[inputNeurons + 1][hiddenNeurons];

	//Context to Hidden weights (with biases).
	wch = MatrixXd(contextNeurons+1, hiddenNeurons);;//[contextNeurons + 1][hiddenNeurons];

	//Hidden to Output weights (with biases).
	who = MatrixXd(hiddenNeurons+1, outputNeurons);;//[hiddenNeurons + 1][outputNeurons];

	//Hidden to Context weights (no biases).
	whc = MatrixXd(outputNeurons+1, contextNeurons);;//[outputNeurons + 1][contextNeurons];

	inputs = VectorXd(inputNeurons);//[inputNeurons];
	hidden = VectorXd(hiddenNeurons);//[hiddenNeurons];
	target = VectorXd(outputNeurons);//[outputNeurons];
	actual = VectorXd(outputNeurons);//[outputNeurons];
	context = VectorXd(contextNeurons);//[contextNeurons];

	//Unit errors.
	erro = VectorXd(outputNeurons);//[outputNeurons];
	errh = VectorXd(hiddenNeurons);//[hiddenNeurons];
	
	//Initialise the output stream
	errO = new std::ofstream("../Predictor/Data/err.txt", std::ofstream::out);
	mseO = new std::ofstream("../Predictor/Data/MSE.txt", std::ofstream::out);
	inO = new std::ofstream("../Predictor/Data/in.txt", std::ofstream::out);
	outO = new std::ofstream("../Predictor/Data/out.txt", std::ofstream::out);
	
}


/**
 * Yarp configure
 */
bool Predictor::configure(yarp::os::ResourceFinder &rf) {    
    
	//init the network
	Network::init();
	
	initPorts(rf);
	
	cout << fixed << setprecision(3) << endl;           //Format all the output.
	srand((unsigned)time(0));   //Seed random number generator with system time.
    
	assignRandomWeights();
	
	return true;
}

/**
 * Assign value to the input
 */
void Predictor::mapInput(double value, int index){
					
	if(std::isnan(value)){
		//If no input, the input is NULL and tha target is NaN (no learning)
		sampleInput(1, index) = 0;	
		sampleInput(0, index) = std::numeric_limits<double>::quiet_NaN();
	}else{	
		if(std::isnan(sampleInput(0, index)))
			sampleInput(1, index) = actual(index);
		else
			sampleInput(1, index) = sampleInput(0, index);
		sampleInput(0, index) = value;	
		//cout<<value<<endl;
	}
}

/**
 * Use files to train and test the network
 */
void Predictor::openFile(string src, int inter){
	bool test = false;
	bool train = false;
	
	//TEST
	ifstream inFile1;
	
	for(int i = 0; i < inter; i ++){
		inFile1.open(src);
		if (inFile1.is_open())
		{
			string line;
			while ( getline (inFile1,line) )
			{
				double value;
				int count = 0;
				
				test = (line.find("TEST") != std::string::npos);
				train = (line.find("LEARN") != std::string::npos);
				
				line = line.substr(line.find(" ")+1);
				while(line.find(" ") != std::string::npos){
					value = string_to_double(line.substr(0, line.find(" ")+1));	
					
					mapInput(value, count);
					
					line = line.substr(line.find(" ")+1);			
					count++;
				}
				mapInput(string_to_double(line), count);
				  
				if(train)
					ElmanNetwork();
				if(test)
					testNetwork();
				
			}
			inFile1.close();
		}else{
			cerr<<"Error while opening input file"<<endl;
		}
		resetNodes();
	}
}

/**
 * Init yarp port
 */
bool Predictor::initPorts(yarp::os::ResourceFinder &rf){
	moduleName            = rf.check("name", 
			      Value("Predictor"), 
			      "module name (string)").asString();
	    /*
	* before continuing, set the module name before getting any other parameters, 
	* specifically the port names which are dependent on the module name
	*/
	setName(moduleName.c_str());

	
	// Open OUT
	portOutName = "/";
	portOutName += getName() + "/out";

	if (!portOut.open(portOutName.c_str())) {           
		cout << getName() << ": Unable to open port " << portOutName << endl;  
		return false;
	}

	
	// Open In
	portInName = "/";
	portInName += getName() + "/in";

	if (!portIn.open(portInName.c_str())) {           
		cout << getName() << ": Unable to open port " << portInName << endl;  
		return false;
	}
	
	
	attach(portIn);

	return true;
}


/**
 * Yarp respond method
 */
bool Predictor::respond(const Bottle& command, Bottle& reply) {
      
    return true;
}


/**
 * Train the network
 */
void Predictor::ElmanNetwork(){
      double err, mse;
      int counter = 0;
      bool sameInput = true;

      //Test is the input is repeted
      for(int i = 0; i <= (inputNeurons - 1); i++){
		if(sampleInput(1, i) != sampleInput(0, i)){
			sameInput = false;
			break;
		}
      }
      
      //If the input is not repeted, train the network and measure the error
      if(!sameInput){
		if(iterations == 0){
		    for(int i = 0; i <= (inputNeurons - 1); i++){
			inputs(i) = beVector(i);
		    } // i
		} else {
		    for(int i = 0; i <= (inputNeurons - 1); i++){
			inputs(i) = sampleInput(1, i);
		    } // i
		}
		
		for(int i = 0; i <= (inputNeurons - 1); i++){
		    target(i) = sampleInput(0, i);
		} // i
		//}
		

		feedForward();
		err = 0.0;
		mse = 0.0;
		for(int i = 0; i <= (inputNeurons - 1); i++){ 
		    if(!std::isnan(target(i))){
			    err += abs(target(i) - actual(i));
			    mse += pow(target(i) - actual(i), 2);
			    //MEas square error for each output
			    *mseO << pow(target(i) - actual(i), 2) << "\t";
			    counter++;
		    }else{
			    *mseO << "NaN" << "\t";
		    }
		} // i		
		
		*mseO<<endl;
		
		//Average error
		*errO<<(double)err/counter<<endl;
		
		backPropagate();
		
	        cout << "Iterations = " << iterations << endl;
		iterations += 1;
	}
}


/**
 * Test the network
 */
void Predictor::testNetwork(){
	bool sameInput = true;
	//for(int test = 0; test <= maxTests; test++){
	    
	for(int i = 0; i <= (inputNeurons - 1); i++){
	      if(sampleInput(1, i) != sampleInput(0, i)){
		      sameInput = false;
		      break;
	      }
	}
	if(!sameInput){
	      //Enter Beginning string.
	      for(int i = 0; i <= (inputNeurons - 1); i++){
		  inputs(i) = (double)sampleInput(1, i);
	      } // i

	      //loop to predict step t+i
	      *inO<<testIter<< "\t";
	      *outO<<testIter<< "\t";
	      for(int i = 0; i < 1; i++){
		      feedForward();
		      for(int i = 0; i <= (inputNeurons - 1); i++){
			  *inO << inputs(i) << "\t";
		      }
		      *inO << endl;
		      
		      for(int i = 0; i <= (inputNeurons - 1); i++){
			  *outO << actual(i) << "\t";
		      } // i
		      *outO << endl;
		      
		    //Output becomes input
		    for(int i = 0; i <= (inputNeurons - 1); i++){
			inputs(i) = actual(i);
		    } // i

	      }
	      testIter++;
	}
//     cout<<"Predicted output is: "<<predicted<<endl;
}


/**
 * Reset all the node to 0
 */
void Predictor::resetNodes(){
      
    for(int in = 0; in <= (inputNeurons - 1); in++){
	    sampleInput(0, in) = 0;
	    sampleInput(1, in) = 0;
    }
    
    for(int out = 0; out <= (outputNeurons - 1); out++){
	    actual(out) = 0;
    }
     
    for(int hid = 0; hid <= (hiddenNeurons - 1); hid++){
	    hidden(hid) = 0;
    }
     
    for(int con = 0; con <= (contextNeurons - 1); con++){
	    context(con) = 0;
    }
}


/**
 * Calculate the value of the neurons in the different layers
 */
void Predictor::feedForward(){
    double sum;

    //Calculate input and context connections to hidden layer.
    for(int hid = 0; hid <= (hiddenNeurons - 1); hid++){
        sum = 0.0;
        //from input to hidden...
        for(int inp = 0; inp <= (inputNeurons - 1); inp++){
	    if(!std::isnan(inputs(inp))){
		    sum += inputs(inp) * wih(inp, hid);
    //  		cout<<"Error = "<<target(out)<<endl;
	    }else{
		    sum+= 0;
    // 		cout<<"no backPropagate for out "<<out<<endl;
	    }
        } // inp
        //from context to hidden...
        for(int con = 0; con <= (contextNeurons - 1); con++){
            sum += context(con) * wch(con, hid);
        } // con
        //Add in bias.
        sum += wih(inputNeurons, hid);
        sum += wch(contextNeurons, hid);
        hidden(hid) = sigmoid(sum);
    } // hid

    //Calculate the hidden to output layer.
    for(int out = 0; out <= (outputNeurons - 1); out++){
        sum = 0.0;
        for(int hid = 0; hid <= (hiddenNeurons - 1); hid++){
            sum += hidden(hid) * who(hid, out);
        } // hid

        //Add in bias.
        sum += who(hiddenNeurons, out);
        actual(out)= sigmoid(sum);
    } // out

    //Copy outputs of the hidden to context layer.
    for(int con = 0; con <= (contextNeurons - 1); con++){
        context(con) = hidden(con);
    } // con

}


/**
 * Train the weight using backpropagation through time
 */
void Predictor::backPropagate(){

    //Calculate the output layer error (step 3 for output cell).
    for(int out = 0; out <= (outputNeurons - 1); out++){
	if(!std::isnan(target(out))){
		erro(out)= (target(out)- actual(out)) * sigmoidDerivative(actual(out));
//  		cout<<"Error = "<<target(out)<<endl;
	}else{
		erro(out) = 0;
// 		cout<<"no backPropagate for out "<<out<<endl;
	}
    } // out

    //Calculate the hidden layer error (step 3 for hidden cell).
    for(int hid = 0; hid <= (hiddenNeurons - 1); hid++){
        errh(hid)= 0.0;
        for(int out = 0; out <= (outputNeurons - 1); out++){
            errh(hid)+= erro(out)* who(hid, out);
        } // out
        errh(hid)*= sigmoidDerivative(hidden(hid));
    } // hid

    //Update the weights for the output layer (step 4).
    for(int out = 0; out <= (outputNeurons - 1); out++){
        for(int hid = 0; hid <= (hiddenNeurons - 1); hid++){
            who(hid, out)+= (learnRate * erro(out)* hidden(hid));
        } // hid
        //Update the bias.
        who(hiddenNeurons, out)+= (learnRate * erro(out));
    } // out

    //Update the weights for the hidden layer (step 4).
    for(int hid = 0; hid <= (hiddenNeurons - 1); hid++){
        for(int inp = 0; inp <= (inputNeurons - 1); inp++){
            wih(inp, hid)+= (learnRate * errh(hid)* inputs(inp));
        } // inp
        //Update the bias.
        wih(inputNeurons, hid)+= (learnRate * errh(hid));
    } // hid

}


/**
 * Assign random weight between the different layers
 */
void Predictor::assignRandomWeights(){

    for(int inp = 0; inp <= inputNeurons; inp++){
        for(int hid = 0; hid <= (hiddenNeurons - 1); hid++){
            //Assign a random weight value between -0.5 and 0.5
            wih(inp, hid)= -0.5 + double(rand()/(RAND_MAX + 1.0));
        } // hid
    } // inp

    for(int con = 0; con <= contextNeurons; con++){
        for(int hid = 0; hid <= (hiddenNeurons - 1); hid++){
            //Assign a random weight value between -0.5 and 0.5
            wch(con, hid)= -0.5 + double(rand()/(RAND_MAX + 1.0));
        } // hid
    } // con

    for(int hid = 0; hid <= hiddenNeurons; hid++){
        for(int out = 0; out <= (outputNeurons - 1); out++){
            //Assign a weight value of 0 to the output weight
            who(hid, out)= 0;
        } // out
    } // hid

    //whc is never used here used
    for(int out = 0; out <= outputNeurons; out++){
        for(int con = 0; con <= (contextNeurons - 1); con++){
            //These are all fixed weights set to 0.5
            whc(out, con)= 0.5;
        } // con
    } // out
    
}

/* Called periodically every getPeriod() seconds */
bool Predictor::updateModule() {
	
    return true;
}

double Predictor::getPeriod() {
    /* module periodicity (seconds), called implicitly by myModule */    
    return 0.05;
}

bool Predictor::interruptModule() {
    return true;
}

bool Predictor::close() {
	
    errO->close();
    inO->close();
    outO->close();
    cout<<"Module closing"<<endl;
    return true;
}


/**
 * getRandomNumber
 */
int Predictor::getRandomNumber(){
    //Generate random value between 0 and 6.
    return int(6*rand()/(RAND_MAX + 1.0));
}

/**
 * Apply sigmoid function to the value
 */
double Predictor::sigmoid(double val){
    return (1.0 / (1.0 + exp(-val)));
}

/**
 * Apply sigmoid derivative function to the value
 */
double Predictor::sigmoidDerivative(double val){
    return (val * (1.0 - val));
}

/**
 * Convert string to double
 * In: 
 * const std::string s: string to convert to double
 * Out: double contained the argument, "nan" if string not a number
 * */
double Predictor::string_to_double(const std::string& s){
   std::istringstream i(s);
   double x;
   if (!(i >> x))
     return std::numeric_limits<double>::quiet_NaN();
   return x;
 } 



/**
 * Apply gaussian function to the value
 */
double Predictor::gaussian(double in, double mean, double std)
{
      return (1./(sqrt(pow(std, 2)*2*M_PI))*exp(-0.5*pow((in-mean)/pow(std, 2), 2)));
}

Predictor::~Predictor(){
  
}

