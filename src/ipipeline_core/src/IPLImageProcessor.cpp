#include <sys/time.h>
#include <time.h>
#include <stdarg.h>


#include "ipipeline_core/IPLImageProcessor.h"
#include "ipipeline_core/IPLIPScheduler.h"

unsigned int IPLImageProcessor::idcounter = 0;

//#define TRACE_INIT

IPLImageProcessor::IPLImageProcessor(const char * n, IPLImageProcessorRole r) :
	frameid(_frameid), name(strdup(n)), role(r), triggerCallback(NULL)
{
#ifdef TRACE_INIT
	printf("Creating %03d %s\n",id,name);
#endif // TRACE_INIT
	id = idcounter++;
	_frameid = 0;
	printed = false;
	condition = NULL;
	condtext = NULL;
	destroyCondition = false;
	passthroughid = 0;
	profiling_flag = NULL;
	profiling_total = 0.0;
	profiling_numruns = 0;
}

IPLImageProcessor::IPLImageProcessor(const char * n, IPLImageProcessor * in, IPLImageProcessorRole r) :
	frameid(_frameid), name(strdup(n)), role(r), triggerCallback(NULL)
{
#ifdef TRACE_INIT
	printf("Creating %03d %s\n",id,name);
#endif // TRACE_INIT
	id = idcounter++;
	_frameid = 0;
	printed = false;
	condition = NULL;
	condtext = NULL;
	destroyCondition = false;
	passthroughid = 0;
	profiling_flag = NULL;
	profiling_total = 0.0;
	profiling_numruns = 0;
}


IPLImageProcessor::~IPLImageProcessor() 
{
#ifdef TRACE_INIT
	printf("Destroying %03d %s\n",id,name);
#endif // TRACE_INIT
	free(name);free(condtext);
	if ((condition != NULL) && destroyCondition)
		delete condition;
}



void IPLImageProcessor::print(FILE * fp) {
	fprintf(fp,"%03d -IP %s : %d images / frame %d\n",id,
			name,(int)outputVector.size(),(int)frameid);
}

void IPLImageProcessor::setFrameId(unsigned int i) {
	_frameid = i;
}


void IPLImageProcessor::setCondition(const char * text, Condition * cond, bool destroyIt) {
	if ((condition != NULL) && destroyCondition)
		delete condition;
	free(condtext);
	condtext = strdup(text);
	destroyCondition = destroyIt;
	condition = cond;
}

void IPLImageProcessor::setCondition(const char * text, const bool & v) {
	if ((condition != NULL) && destroyCondition)
		delete condition;
	free(condtext);
	condtext = strdup(text);
	destroyCondition = true;
	condition = new BoolCondition(v);
}

void IPLImageProcessor::setCondition(const char * text, const signed int & v) {
	if ((condition != NULL) && destroyCondition)
		delete condition;
	free(condtext);
	condtext = strdup(text);
	destroyCondition = true;
	condition = new NeqCondition<signed int>(v,0);
}

void IPLImageProcessor::setCondition(const char * text, const unsigned int & v) {
	if ((condition != NULL) && destroyCondition)
		delete condition;
	free(condtext);
	condtext = strdup(text);
	destroyCondition = true;
	condition = new NeqCondition<unsigned int>(v,0);
}

void IPLImageProcessor::setCondition(const IPLImageProcessor * master)
{
	setCondition(master->condtext,master->condition,false);
}

bool IPLImageProcessor::registerInput(
		IPLImageProcessor * input,
		unsigned int inputid)
{
	if (input != NULL) {
		setInput(inputid,input);
		if (inputid == 0) /* take frameid from first input */
			setFrameId(input->frameid);
		assert(inputs[inputid].received == input);
	}
	
	// This should have been checked by the scheduler
	if (!hasReceivedAllInputs()) {
		//printf("not yet\n"); fflush(stdout);
		return false;
	}
	//printf("go\n"); fflush(stdout);

	return true;
}

void IPLImageProcessor::testAndProcessInput(
		IPLIPScheduler * scheduler)
{
	unsigned int i;

	if ((condition == NULL) || (condition->isVerified())) {
		assert(checkInput());
		double t1=0,t2=0;
		if ((profiling_flag!=NULL) && *profiling_flag)
			t1 = timeofday();
		
		// the real computation take place here...
		processInput();
		
		if ((profiling_flag!=NULL) && *profiling_flag) {
			t2 = timeofday();
			profiling_total += t2-t1;
			profiling_numruns += 1;
		}
		for (i=0;i<receiver.size();i++) {
			scheduler->push(receiver[i].first,this,receiver[i].second);
		}
	} else {
		IPLImageProcessor * firstinput = getInput(passthroughid);
		for (i=0;i<receiver.size();i++) {
			scheduler->push(receiver[i].first,firstinput,receiver[i].second);
		}
	}
	resetInputsReceived();
	//checkState();
}

void IPLImageProcessor::removeReceiver(IPLImageProcessor * rec)
{
	rec->inputs[0].connected = false;
	std::vector<Receiver>::iterator it;
	for (it=receiver.begin();it!=receiver.end();it++){
		if (it->first == rec) {
			receiver.erase(it);
			break;
		}
	}
}

void IPLImageProcessor::addReceiver(IPLImageProcessor * rec, 
		unsigned int inputid)
{
	assert(inputid < rec->inputs.size());
	assert(!rec->inputs[inputid].connected);
	rec->inputs[inputid].connected = true;
	receiver.push_back(Receiver(rec,inputid));
}

void IPLImageProcessor::addReceiver(IPLImageProcessor * rec, 
		const char * inputname)
{
	int inputid = rec->getInputId(inputname);
	assert(inputid  >= 0);
	assert(!rec->inputs[inputid].connected);
	rec->inputs[inputid].connected = true;
	receiver.push_back(Receiver(rec,inputid));
}


unsigned int IPLImageProcessor::addInput(const char * iname)
{
	int id = getInputId(iname);
	if (id > 0) {
		return id;
	} 
	id = inputs.size();
	inputs.push_back(Input(iname));
	//printf("Added input %s in %s (%d)\n", iname,name,id);
	return id;
}

void IPLImageProcessor::setInputName(unsigned int id, const char * iname)
{
	assert (id < inputs.size());
	inputs[id] = Input(iname);
}

IPLImageProcessor * IPLImageProcessor::getInput(const char * name) const
{
	int id = getInputId(name);
	if (id < 0) return NULL;
	return inputs[id].received;
}

IPLImageProcessor * IPLImageProcessor::getInput(unsigned int id) const
{
	assert (id < inputs.size());
	return inputs[id].received;
}

void IPLImageProcessor::setInput(unsigned int id, IPLImageProcessor * input)
{
	assert(id < inputs.size());
	assert(inputs[id].received == NULL);
	inputs[id].received = input;
}

int IPLImageProcessor::getInputId(const char * name) const
{
	//string sname(name);
	unsigned int i;
	for (i=0;i<inputs.size();i++)
		if (strcasecmp(inputs[i].name.c_str(),name)==0)
			return i;
	return -1;
}

void IPLImageProcessor::resetInputsReceived()
{
	//printf("Resetting %s\n",name);
	unsigned int i;
	for (i=0;i<inputs.size();i++)
		inputs[i].reset();
}

bool IPLImageProcessor::hasReceivedAllInputs() const
{
	unsigned int i;
	for (i=0;i<inputs.size();i++)
		if (inputs[i].received == NULL) 
			return false;
	return true;
}

bool IPLImageProcessor::checkProcessingTree() const
{
	unsigned int i;
	for (i=0;i<inputs.size();i++) {
		if (!inputs[i].connected) {
			fprintf(stderr,"In module '%s' (%d), input '%s' (%d) is not connected\n",
					name, id, inputs[i].name.c_str(),i);
			return false;
		}
	}
	for (i=0;i<receiver.size();i++) {
		if (!receiver[i].first->checkProcessingTree()) return false;
	}
	return true;
}


ImageProcessorOutput IPLImageProcessor::getOutput() const
{
    return outputVector;
}

void IPLImageProcessor::reallocate(int type, unsigned int w, unsigned int h, unsigned int nimg)
{
    unsigned int i;
    cv::Size sz(w,h);
    outputVector.resize(nimg);
    for (i=0;i<outputVector.size();i++) {
        outputVector[i].create(sz,type);
    }
}

void IPLImageProcessor::reallocate(const std::vector<cv::Mat> & vin)
{
    unsigned int i;
    outputVector.resize(vin.size());
    for (i=0;i<outputVector.size();i++) {
        outputVector[i].create(vin[i].size(),vin[i].type());
    }
}

void IPLImageProcessor::reallocate(const cv::Mat & min)
{
    outputVector.resize(1);
    outputVector[0].create(min.size(),min.type());
}


void IPLImageProcessor::recPreparePrint()
{
	unsigned int i;
	printed=false;
	for (i=0;i<receiver.size();i++)
		receiver[i].first->recPreparePrint();
}

void IPLImageProcessor::recPrintTree(FILE * fp, unsigned int indent)
{
	unsigned int i;
	for (i=0;i<indent;i++)
		fprintf(fp,"  ");
	fprintf(fp,"+");
	print(fp);
	if (printed) return;
	printed = true;
	for (i=0;i<receiver.size();i++)
		receiver[i].first->recPrintTree(fp,indent+1);
}

void IPLImageProcessor::printTree(FILE * fp)
{
	recPreparePrint();
	recPrintTree(fp,0);
}

void IPLImageProcessor::printVCG(const char * fname,
		OpStore & sources, OpStore & blacklist)
{
	FILE * fp = stdout;
	if (fname != NULL) {
		fp = fopen(fname,"w");
		if (fp == NULL) fp = stdout;
	}
	fprintf(fp,"graph:{\n");
	fprintf(fp,"title:\"Image Processing\"\n");
	fprintf(fp,"node.shape: box\n");
	fprintf(fp,"display_edge_labels: yes\n");
	fprintf(fp,"manhatten_edges: yes\n");
	fprintf(fp,"late_edge_labels: no\n");
	fprintf(fp,"dirty_edge_labels: no\n");
	fprintf(fp,"finetuning: yes\n");
	fprintf(fp,"nearedges: no\n");
	fprintf(fp,"splines: yes\n");
	fprintf(fp,"port_sharing: no\n");
	fprintf(fp,"crossingphase2: no\n");
	fprintf(fp,"crossingoptimization: yes\n");
	fprintf(fp,"yspace: 15\n");
	fprintf(fp,"xspace: 50\n");

	OpStore::iterator i;
	// Preparing the processing tree to be printed
	for (i=sources.begin();i!=sources.end();i++) {
		IPLImageProcessor * ip = *i;
		ip->recPreparePrint(); 
	}
	
	// printed the nodes
	for (i=sources.begin();i!=sources.end();i++) {
		IPLImageProcessor * ip = *i;
		ip->recPrintVCG(fp,false,blacklist); 
	}

	// Preparing the processing tree to be re-printed
	for (i=sources.begin();i!=sources.end();i++) {
		IPLImageProcessor * ip = *i;
		ip->recPreparePrint(); 
	}
	// printed the edges
	for (i=sources.begin();i!=sources.end();i++) {
		IPLImageProcessor * ip = *i;
		ip->recPrintVCG(fp,true,blacklist); 
	}

	fprintf(fp,"\n}");
	if (fp != stdout) fclose(fp);
}

void IPLImageProcessor::printVCG(const char * fname)
{
	OpStore sources, opprobes;
	sources.insert(this);
	printVCG(fname,sources,opprobes);
}

std::string IPLImageProcessor::createOutputTypeStr(const cv::Mat & m) const
{
    char tmp[64];
    std::string txt;
    switch (m.type() & CV_MAT_DEPTH_MASK) {
        case CV_8U  : txt += "8UC" ; break;
        case CV_8S  : txt += "8SC" ; break;
        case CV_16U : txt += "16UC"; break;
        case CV_16S : txt += "16SC"; break;
        case CV_32S : txt += "32SC"; break;
        case CV_32F : txt += "32FC"; break;
        case CV_64F : txt += "64FC"; break;
        default: break;
    }
    sprintf(tmp,"%d: %dx%d",m.type() >> CV_CN_SHIFT,m.cols,m.rows);
    txt += tmp;
    return txt;
}

std::string IPLImageProcessor::createOutputTypeStr() const 
{
    unsigned int i;
    ImageProcessorOutput out = this->getOutput();
    if (out.size() == 0) {
        return "undefined";
    }
    if (out.size() == 1) {
        return createOutputTypeStr(out[0]);
    }
    std::string txt = "[";
    for (i=0;i<out.size()-1;i++) {
        txt += createOutputTypeStr(out[i]);
        txt += ",";
    }
    txt += createOutputTypeStr(out[i]) + "]";
    return txt;
}

void IPLImageProcessor::recPrintVCG(FILE * fp, bool edge, 
		const OpStore & blacklist)
{
	unsigned int i;
	if (printed) return;
	if (blacklist.find(this) != blacklist.end()) return;
	printed = true;
	if (edge) {
		for (i=0;i<receiver.size();i++) {
			if (blacklist.find(receiver[i].first) != blacklist.end())
				continue;
			fprintf(fp,"edge: {sourcename: \"%03d\" targetname: \"%03d\" label: \"%s\"}\n",
					id,	receiver[i].first->id,
					createOutputTypeStr().c_str());
		}
	} else /* node */ if (condition != NULL) {
		fprintf(fp,"node: {title: \"%03d\" label: \"%s\\ncond: %s\"}\n",id,name,condtext);
	} else {
		fprintf(fp,"node: {title: \"%03d\" label: \"%s\"}\n",id,name);
	}
	for (i=0;i<receiver.size();i++)
		receiver[i].first->recPrintVCG(fp,edge,blacklist);
}

void IPLImageProcessor::printDOT(const char * fname)
{
	OpStore sources, opprobes;
	sources.insert(this);
	printDOT(fname,sources,opprobes);
}

void IPLImageProcessor::printDOT(const char * fname,
		OpStore & sources, OpStore & blacklist)
{
	FILE * fp = stdout;
	if (fname != NULL) {
		fp = fopen(fname,"w");
		if (fp == NULL) fp = stdout;
	}
	fprintf(fp,"digraph \"Image Processing\" {\n");
	fprintf(fp,"node [shape=box]\n");


	OpStore::iterator i;
	for (i=sources.begin();i!=sources.end();i++) {
		IPLImageProcessor * ip = *i;
		ip->recPreparePrint(); 
	}
	for (i=sources.begin();i!=sources.end();i++) {
		IPLImageProcessor * ip = *i;
		ip->recPrintDOT(fp,false,blacklist); 
	}

	for (i=sources.begin();i!=sources.end();i++) {
		IPLImageProcessor * ip = *i;
		ip->recPreparePrint(); 
	}
	for (i=sources.begin();i!=sources.end();i++) {
		IPLImageProcessor * ip = *i;
		ip->recPrintDOT(fp,true,blacklist); 
	}

	fprintf(fp,"\n}");
	if (fp != stdout) fclose(fp);
}

void IPLImageProcessor::recPrintDOT(FILE * fp, bool edge, 
		const OpStore & blacklist)
{
	unsigned int i;
	if (printed) return;
	if (blacklist.find(this) != blacklist.end()) return;
	printed = true;
	if (edge) {
		for (i=0;i<receiver.size();i++){
			if (blacklist.find(receiver[i].first) != blacklist.end())
				continue;
			fprintf(fp,"%03d -> %03d [label=\"%s\"]\n",
					id,	receiver[i].first->id,
                    createOutputTypeStr().c_str());
		}
	} else /* node */ if (condition != NULL) {
		fprintf(fp,"%03d [label=\"%s\\ncond: %s\"]\n",id,name,condtext);
	} else {
		fprintf(fp,"%03d [label=\"%s\"]\n",id,name);
	}
	for (i=0;i<receiver.size();i++)
		receiver[i].first->recPrintDOT(fp,edge,blacklist);
}

void IPLImageProcessor::setPassThrough(const char * inputname)
{
	int id = getInputId(inputname);
	assert(id >= 0);
	if (id < 0) return;
	passthroughid = id;
}

double IPLImageProcessor::timeofday()
{
	struct timeval tv;
	gettimeofday(&tv,NULL);
	return tv.tv_sec + 1e-6*tv.tv_usec;
}

void IPLImageProcessor::checkState() const
{
	unsigned int i;
	for (i=0;i<inputs.size();i++) assert(inputs[i].received == NULL);
}

bool IPLImageProcessor::error(const char * text,...) const
{
	va_list args;
    va_list args_copy;
	va_start(args,text);  
    va_copy(args_copy, args);

    std::string out = std::string(name) + ": ";
    char buffer[128];
	size_t total = vsnprintf(buffer,128,text,args);
    if (total > 128) {
        char bigbuffer[total];
        vsnprintf(bigbuffer,128,text,args_copy);
        out += bigbuffer;
    } else {
        out += buffer;
    }
	va_end(args_copy);
	va_end(args);
    ROS_ERROR("%s", out.c_str());
	return false;
}

bool IPLImageProcessor::error(ErrorMessageId id,const std::string & extra) const
{
    std::string text = "Unknown error id";
	switch (id) {
		case INCONSISTENT_TYPE: text = ("Inconsistent input type"); break;
		case INVALID_TYPE: text = ("Invalid input type"); break;
		case INVALID_IMAGE_SIZE: text = ("Invalid input image size"); break;
		case INVALID_INPUT_SIZE: text = ("Invalid input size"); break;
		case INCONSISTENT_INPUT_SIZE: text = ("Inconsistent input size"); break;
		default : break;
	}
    if (extra.empty()) {
        return error(text.c_str());
    } else {
        return error((text+": "+extra).c_str());
    }
}

void IPLImageProcessor::readConfig(Config * config)
{
	/** Nothing to do by default **/
	printf("Warning: '%s' config data ignored\n",name);
}

template <class elem>
bool tsaveMatToFile1C(const cv::Mat & m, const std::string & filename) {
    const cv::Mat_<elem> & I = (const cv::Mat_<elem> &)m;
    FILE * fp = fopen(filename.c_str(),"w");
    std::string templ = "%d ";
    if (!fp) {
        perror("IPLImageProcessor::saveMatToFile:");
        return false;
    }
    if ((I.type() & CV_MAT_DEPTH_MASK) >= CV_32F) {
        templ = "%e ";
    }
    assert(I.channels() == 1);
    for (int v=0;v<I.rows;v++) {
        for (int u=0;u<I.cols;u++) {
            fprintf(fp,templ.c_str(),I(v,u));
        }
        fprintf(fp,"\n");
    }
    fclose(fp);
    return true;
}

template <class elem>
bool tsaveMatToFileMC(const cv::Mat & m, const std::string & filename) {
    const cv::Mat_<elem> & I = (const cv::Mat_<elem> &)m;
    FILE * fp = fopen(filename.c_str(),"w");
    std::string templ = "%d ";
    if (!fp) {
        perror("IPLImageProcessor::saveMatToFile:");
        return false;
    }
    if ((I.type() & CV_MAT_DEPTH_MASK) >= CV_32F) {
        templ = "%e ";
    }
    unsigned int chann = I.channels();
    assert(chann > 1);
    for (int v=0;v<I.rows;v++) {
        for (int u=0;u<I.cols;u++) {
            const elem & P = I(u,v); 
            fprintf(fp, "{");
            for (unsigned int k=0;k<chann;k++) {
                fprintf(fp,templ.c_str(),P[k]);
            }
            fprintf(fp, "} ");
        }
        fprintf(fp,"\n");
    }
    fclose(fp);
    return true;
}


bool IPLImageProcessor::saveMatToFile(const cv::Mat & mat, const std::string & filename)
{
	switch (mat.type()) {
		case CV_8UC1: return tsaveMatToFile1C<uchar>(mat,filename);
        case CV_8UC2: return tsaveMatToFileMC<cv::Vec2b>(mat,filename);
		case CV_8UC3: return tsaveMatToFileMC<cv::Vec3b>(mat,filename);
		case CV_8UC4: return tsaveMatToFileMC<cv::Vec4b>(mat,filename);

		case CV_16UC1: return tsaveMatToFile1C<ushort>(mat,filename);
        case CV_16UC2: return tsaveMatToFileMC<cv::Vec2w>(mat,filename);
		case CV_16UC3: return tsaveMatToFileMC<cv::Vec3w>(mat,filename);
		case CV_16UC4: return tsaveMatToFileMC<cv::Vec4w>(mat,filename);

		case CV_16SC1: return tsaveMatToFile1C<short>(mat,filename);
        case CV_16SC2: return tsaveMatToFileMC<cv::Vec2s>(mat,filename);
		case CV_16SC3: return tsaveMatToFileMC<cv::Vec3s>(mat,filename);
		case CV_16SC4: return tsaveMatToFileMC<cv::Vec4s>(mat,filename);

		case CV_32SC1: return tsaveMatToFile1C<int>(mat,filename);
        case CV_32SC2: return tsaveMatToFileMC<cv::Vec2i>(mat,filename);
		case CV_32SC3: return tsaveMatToFileMC<cv::Vec3i>(mat,filename);
		case CV_32SC4: return tsaveMatToFileMC<cv::Vec4i>(mat,filename);

		case CV_32FC1: return tsaveMatToFile1C<float>(mat,filename);
        case CV_32FC2: return tsaveMatToFileMC<cv::Vec2f>(mat,filename);
		case CV_32FC3: return tsaveMatToFileMC<cv::Vec3f>(mat,filename);
		case CV_32FC4: return tsaveMatToFileMC<cv::Vec4f>(mat,filename);

		case CV_64FC1: return tsaveMatToFile1C<double>(mat,filename);
        case CV_64FC2: return tsaveMatToFileMC<cv::Vec2d>(mat,filename);
		case CV_64FC3: return tsaveMatToFileMC<cv::Vec3d>(mat,filename);
		case CV_64FC4: return tsaveMatToFileMC<cv::Vec4d>(mat,filename);
        default: return false;
    }
}




