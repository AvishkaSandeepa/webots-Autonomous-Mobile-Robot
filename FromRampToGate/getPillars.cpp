double getPillars() {
    //cout << "show pillars" << endl;
    DistanceSensor*RightDS;
    DistanceSensor*LeftDS;
 
    
    RS=robot->getDistanceSensor("RightDS");
    LS=robot->getDistanceSensor("LeftDS");

    
    RS->enable(TIME_STEP);
    LS->enable(TIME_STEP);
    
    double R_DS=RS->getValue();
    double L_DS=LS->getValue();

    return R_DS,L_DS
    
    
    cout <<"LS- "<<L_DS<<"      RS- "<<R_DS<< endl;
}