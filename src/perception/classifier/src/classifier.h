#ifndef CLASSIFIER_H
#define CLASSIFIER_H

struct foundObject {
    std::string color;
    std::string type;
	
    float certainty;
	
    float x;
    float y;
    float z;
};

#endif // CLASSIFIER_H
