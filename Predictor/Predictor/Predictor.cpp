/* 
 * Copyright (C): None
 * Authors: Jimmy Baraglia
 * Public License for more details
*/

#define QUEUE_SIZE 5

#include "Predictor.h"


Predictor::Predictor(ResourceFinder &rf):
maxTests(10000), maxSamples(4), inputNeurons(3),
hiddenNeurons(10), outputNeurons(3), contextNeurons(10),
learnRate(0.2), trainingReps(2000), iterations(0)
{    
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
	
	beVector = VectorXd(inputNeurons);//[inputNeurons] = {1.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	beVector << 0.0, 0.0, 0.0;
		    //  0    1    2    3    4    5
	sampleInput = MatrixXd(2, inputNeurons);
	sampleInput <<  0.0, 0.0, 0.0,
			0.0, 0.0, 0.0;

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
	

	
}

bool Predictor::configure(yarp::os::ResourceFinder &rf) {    
    
	//init the network
	Network::init();
	
	initPorts(rf);
	
	cout << fixed << setprecision(3) << endl;           //Format all the output.
	srand((unsigned)time(0));   //Seed random number generator with system time.
    
	assignRandomWeights();
    
	/*ElmanNetwork();
	testNetwork();*/
	
	//TEST
	if(rf.find("train").asInt() == 1){
		for(int i = 0; i < trainingReps/2; i++)
		{
			for(int i = 0; i <= (inputNeurons - 1); i++){
				sampleInput(1, i) = sampleInput(0, i);
				sampleInput(0, i) = 0;
			}
			sampleInput(0, 5) = 1;
			ElmanNetwork();
			
			for(int i = 0; i <= (inputNeurons - 1); i++){
				sampleInput(1, i) = sampleInput(0, i);
				sampleInput(0, i) = 0;
			}
			sampleInput(0, 4) = 1;
			ElmanNetwork();
		}
	}
	
	return true;
}

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
	
    return true;
}

bool Predictor::respond(const Bottle& command, Bottle& reply) {
  
    for(int i = 0; i <= (inputNeurons - 1); i++){
	  sampleInput(1, i) = sampleInput(0, i);
	  sampleInput(0, i) = command.get(i+1).asDouble();
    }
	  
    if(command.get(0).asString() == "learning")
	  ElmanNetwork();
    else {
	  testNetwork();
    }
      
    return true;
}

void Predictor::ElmanNetwork(){
    double err;
    bool stopLoop = false;

    //Train the network.
    
    if(iterations == 0){
	for(int i = 0; i <= (inputNeurons - 1); i++){
	    inputs(i) = beVector(i);
	} // i
    } else {
	for(int i = 0; i <= (inputNeurons - 1); i++){
	    inputs(i) = sampleInput(0, i);
	} // i
    }

    //After the samples are entered into the input units, the sample are
    //then offset by one and entered into target-output units for
    //later comparison.
    /*if(sample == maxSamples - 1){
	for(int i = 0; i <= (inputNeurons - 1); i++){
	    target(i) = beVector(i);
	} // i
    } else {*/
    for(int i = 0; i <= (inputNeurons - 1); i++){
	target(i) = sampleInput(1, i);
    } // i
    //}

    feedForward();
    
    err = 0.0;
    for(int i = 0; i <= (outputNeurons - 1); i++){ 
	err += sqrt(abs(target(i) - actual(i)));
    } // i
    err = err/(outputNeurons);
    cout<<err<<endl;

    /*if(iterations > trainingReps){
	stopLoop = true;
    }*/
    iterations += 1;
    
    backPropagate();

    /*sample += 1;
    if(sample == maxSamples){
	sample = 0;
    }*/

//     cout << "Iterations = " << iterations << endl;
}

void Predictor::testNetwork(){
    int index;
    int randomNumber, predicted;
    bool stopTest, stopSample, successful;

    //Test the network with random input patterns.
    stopTest = false;
    //for(int test = 0; test <= maxTests; test++){
        
    //Enter Beginning string.
    for(int i = 0; i <= (inputNeurons - 1); i++){
	inputs(i) = sampleInput(0, i);
    } // i
    cout << "(0) ";

    feedForward();

    predicted = 0;
    

    for(int i = 0; i <= (inputNeurons-1); i++){
	cout << actual(i) << " ";
	if(actual(i) >= 0.3){
	    //The output unit with the highest value (usually over 0.3)
	    //is the network's predicted unit that it expects to appear
	    //in the next input vector.
	    //For example, if the 3rd output unit has the highest value,
	    //the network expects the 3rd unit in the next input to
	    //be 1.0
	    //If the actual value isn't what it expected, the random
	    //sequence has failed, and a new test sequence begins.
	    predicted = i;
	}
    } // i
    cout << "\n";

    cout<<"Predicted output is: "<<predicted<<endl;
}

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
        context(con)= hidden(con);
    } // con

}

void Predictor::backPropagate(){

    //Calculate the output layer error (step 3 for output cell).
    for(int out = 0; out <= (outputNeurons - 1); out++){
        erro(out)= (target(out)- actual(out)) * sigmoidDerivative(actual(out));
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
            //Assign a random weight value between -0.5 and 0.5
            who(hid, out)= -0.5 + double(rand()/(RAND_MAX + 1.0));
        } // out
    } // hid

    for(int out = 0; out <= outputNeurons; out++){
        for(int con = 0; con <= (contextNeurons - 1); con++){
            //These are all fixed weights set to 0.5
            whc(out, con)= 0.5;
        } // con
    } // out

}

int Predictor::getRandomNumber(){
    //Generate random value between 0 and 6.
    return int(6*rand()/(RAND_MAX + 1.0));
}

double Predictor::sigmoid(double val){
    return (1.0 / (1.0 + exp(-val)));
}

double Predictor::sigmoidDerivative(double val){
    return (val * (1.0 - val));
}

Predictor::~Predictor(){
  
}

