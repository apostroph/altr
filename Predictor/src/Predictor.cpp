/* 
 * Copyright (C): None
 * Authors: Jimmy Baraglia
 * Public License for more details
*/

#define QUEUE_SIZE 5
#define DELTA 150

#include "Predictor.h"


Predictor::Predictor(ResourceFinder &rf):
motorNeurons(4), visualNeurons(6),
hiddenNeurons(40), contextNeurons(40),
learnRate(0.05)
{    
	//Assign paramters values to visual neurons and motor neurons
	if(rf.find("VN").asInt() != 0)
		visualNeurons = rf.find("VN").asInt();// = 6;
	if(rf.find("MN").asInt() != 0)
		motorNeurons = rf.find("MN").asInt();// = 6;
	
	//The number of input and output neuron is equal to the sum of motor and visual neurons
	inputNeurons = visualNeurons+motorNeurons;
	outputNeurons = inputNeurons;
	
	//Assign paramters values to hidden neurons and context neurons
	if(rf.find("HN").asInt() != 0)
		hiddenNeurons = rf.find("HN").asInt();// = 3;
	if(rf.find("CN").asInt() != 0)
		contextNeurons = rf.find("CN").asInt();// = 3;

	//Assign paramters values to the learning rate
	if(rf.find("LRATE").asDouble() != 0.0)
		learnRate = rf.find("LRATE").asDouble();// = 0.2;    //Rho.
	
	//If files are used for training
	FILE = rf.find("FILE").asInt();
	
	//Used for the first iteration
	beVector = VectorXd(inputNeurons);//[inputNeurons] = {1.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	
	//Buffer vectors
	nextInput = VectorXd(inputNeurons);
	nextTarget = VectorXd(inputNeurons);
	
	//Initialise beVector to 0
	for(int i = 0; i <= (inputNeurons-1); i++){
		beVector(i) = 0;
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
	
	inO = new std::ofstream("../Predictor/Data/In.cvs");
	outO = new std::ofstream("../Predictor/Data/Out.cvs");
	//Mean error over DELTA values
	EmO = new std::ofstream("../Predictor/Data/Em.cvs");
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
    
	
	resetNodes();
	assignRandomWeights();
	iterations = 0;
	
	return true;
}

/**
 * Assign value to the input
 */
void Predictor::mapInput(double value, int index){
					
	if(std::isnan(value)){
		//If no input, the input is NULL and tha target is NaN (no learning)
		nextInput(index) = 0;
		nextTarget(index) = -1;
	}else{	
		if(nextTarget(index) != -1)
			nextInput(index) = nextTarget(index);
		else
			nextInput(index) = actual(index);
		nextTarget(index) = value;	
		//cout<<value<<endl;
	}
}

/**
 * Use files to train and test the network
 */
void Predictor::openFile(string src, int inter, bool train_test){
	bool test = !train_test;
	bool train = train_test;
	
	maxError = -1;
	
	//TEST
	ifstream inFile1;
	
	for(int i = 0; i < inter; i ++){
		inFile1.open(src);
		if (inFile1.is_open())
		{	
			fileCounter = 0;
			averageError = 0;
			string line;
			while ( getline (inFile1,line) )
			{
				double value;
				int count = 0;
				while(line.find(" ") != std::string::npos){
					value = string_to_double(line.substr(0, line.find(" ")+1));	
					
					mapInput(value, count);
					
					line = line.substr(line.find(" ")+1);			
					count++;
				}
				mapInput(string_to_double(line), count);
				  
				if(train){
					ElmanNetwork();
					fileCounter ++;
				}
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
  
    bool file_learning, test_train;
  
    file_learning =  command.get(0).asString().find("FILE") != std::string::npos;
    test_train =  command.get(1).asString().find("TRAIN") != std::string::npos;
    
    if(command.get(0).asString().find("RESET") != std::string::npos){
	    resetNodes();
	    assignRandomWeights();
	    iterations = 0;
    }
    
    if(file_learning && command.get(4).asString().find(".csv") != std::string::npos){
	inO = new std::ofstream("../Predictor/Data/in"+command.get(4).asString(), std::ofstream::out);
	outO = new std::ofstream("../Predictor/Data/out"+command.get(4).asString(), std::ofstream::out);
	EmO = new std::ofstream("../Predictor/Data/Em"+command.get(4).asString(), std::ofstream::out);
    }
    
    if(file_learning){
	  openFile(command.get(2).asString(), command.get(3).asInt(), test_train);
    }
	
    reply.clear();
    reply.addString("ACK");
      
    return true;
}

void Predictor::measurePE(){
	double err = 0.0;
	int counter = 0;
	
	
	//Measure errors (visual outputs only)
	for(int i = 0; i < (outputNeurons - motorNeurons - 6); i++){ 
		err += abs(target(i) - actual(i));
		counter++;
	} // i		
	//Measure errors (visual outputs only)
	/*for(int i = (outputNeurons - motorNeurons - 2); i < (outputNeurons - motorNeurons); i++){ 
		err += abs(target(i) - actual(i));
		counter++;
	} // i		*/
	
	//Measure average error (visual outputs only)
	averageError = (double)err/(double)counter;
	
	//Add new error value ot the sliding window
	Em.push_back(averageError);
	if(Em.size() > DELTA){
		Em.pop_front();
	}
	
	//Mean error (sliding window size DELTA)
	double Em_t = 0;
	for(auto i: Em){
		Em_t += i;
	}
	
	//Ouput value to the file
	*EmO << (Em_t/Em.size()) << endl;
	      
}


/**
 * Train the network
 */
void Predictor::ElmanNetwork(){
      bool sameInput = true;

      //Test is the input is repeted
      for(int i = 0; i <= (inputNeurons - 1); i++){
		if(nextInput(i) != nextTarget(i) && nextInput(i) != 0){
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
			inputs(i) = nextInput(i);
		    } // i
		}
		
		for(int i = 0; i <= (inputNeurons - 1); i++){
		    target(i) = nextTarget(i);
		} // i
		//}
		

		feedForward();
		
		measurePE();
		
		backPropagate();
		
	        //cout << "Iterations = " << iterations << endl;
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
		if(nextInput(i) != nextTarget(i) && nextInput(i) != 0){
		      sameInput = false;
		      break;
	      }
	}
	if(!sameInput){
	      //Enter Beginning string.
	      for(int i = 0; i <= (inputNeurons - 1); i++){
		  inputs(i) = (double)nextInput(i);
	      } // i
		
	      for(int i = 0; i <= (inputNeurons - 1); i++){
		  target(i) = nextTarget(i);
	      } // i

	      *inO<<testIter<< ",";
	      *outO<<testIter<< ",";
	      
	      feedForward();
	      
	      measurePE();
	      
	      //Save inputs
	      for(int i = 0; i <= (inputNeurons - 1); i++){
		  *inO << inputs(i) << ",";
	      }
	      *inO << endl;
	      
	      //Save outputs
	      for(int i = 0; i <= (inputNeurons - 1); i++){
		  *outO << actual(i) << ",";
	      } // i
	      *outO << endl;
	      
	      testIter++;
	}
//     cout<<"Predicted output is: "<<predicted<<endl;
}


/**
 * Reset all the node to 0
 */
void Predictor::resetNodes(){
      
    for(int in = 0; in <= (inputNeurons - 1); in++){
	    nextTarget(in) = 0;
	    nextInput(in) = 0;
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
		    sum += inputs(inp) * wih(inp, hid);
        } // inp
        
        //from context to hidden...
        for(int con = 0; con <= (contextNeurons - 1); con++){
            sum += context(con) * wch(con, hid);
        } // con
        
        //Add in bias.
       // sum += wih(inputNeurons, hid);
       // sum += wch(contextNeurons, hid);
	
        hidden(hid) = sigmoid(sum);
    } // hid

    //Calculate the hidden to output layer.
    for(int out = 0; out <= (outputNeurons - 1); out++){
        sum = 0.0;
	
        for(int hid = 0; hid <= (hiddenNeurons - 1); hid++){
            sum += hidden(hid) * who(hid, out);
        } // hid

        //Add in bias.
        //sum += who(hiddenNeurons, out);
	
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
	  if(target(out) != -1)
		  erro(out)= (target(out)- actual(out)) * sigmoidDerivative(actual(out));
	  else
		  erro(out) = 0;
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
	      if(target(inp) != -1)
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
            who(hid, out)= -0.5 + double(rand()/(RAND_MAX + 1.0));
        } // out
    } // hid

    //whc is never used here used
    for(int out = 0; out <= outputNeurons; out++){
        for(int con = 0; con <= (contextNeurons - 1); con++){
            //These are all fixed weights set to 0.5
            whc(out, con)= -0.5 + double(rand()/(RAND_MAX + 1.0));
        } // con
    } // out
    
}

/* Called periodically every getPeriod() seconds */
bool Predictor::updateModule() {
    return true;
}

double Predictor::getPeriod() {
    /* module periodicity (seconds), called implicitly by myModule */    
    return 0.50;
}

bool Predictor::interruptModule() {
    return true;
}

bool Predictor::close() {
	
    EmO->close();
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

