class Point{
    PVector pos, vel, force;
    float mass;
 
    Point(PVector pos, PVector vel, PVector force, float mass){
        this.pos = pos;
        this.vel = vel;
        this.force = force;
        this.mass = mass;
    }

    void changeForce(PVector force){
        this.force = force;
    }

    void addForce(PVector force){
        this.force = this.force.add(force);
    }

    void sim(){
        this.vel = this.vel.add(force.mult(delTime));
        this.pos = this.pos.add(vel.mult(delTime));
    }

    PVector getPos(){
        return this.pos;
    }

    PVector getVel(){
        return this.vel;
    }

    PVector getForce(){
        return this.force;
    }
    
    float getMass(){
        return this.mass;
    }
    

  void rungeKuttaStep(float h) {
    // Save current state
    PVector pos0 = pos.copy();
    PVector vel0 = vel.copy();

    // First Runge-Kutta step
    PVector k1v = calcAcceleration();
    PVector k1p = vel.copy();
    k1p.mult(h);

    // Second Runge-Kutta step
    pos.add(k1p.copy().mult(0.5));
    vel.add(k1v.copy().mult(0.5));
    PVector k2v = calcAcceleration();
    PVector k2p = vel.copy();
    k2p.mult(h);

    // Third Runge-Kutta step
    pos = pos0.copy();
    pos.add(k2p.copy().mult(0.5));
    vel = vel0.copy();
    vel.add(k2v.copy().mult(0.5));
    PVector k3v = calcAcceleration();
    PVector k3p = vel.copy();
    k3p.mult(h);

    // Fourth Runge-Kutta step
    pos = pos0.copy();
    pos.add(k3p);
    vel = vel0.copy();
    vel.add(k3v);
    PVector k4v = calcAcceleration();
    PVector k4p = vel.copy();
    k4p.mult(h);

    // Final state update
    pos = pos0.copy();
    pos.add((k1p.copy().add(k2p.copy().mult(2)).add(k3p.copy().mult(2)).add(k4p.copy())).mult(1.0 / 6));

    vel = vel0.copy();
    vel.add((k1v.copy().add(k2v.copy().mult(2)).add(k3v.copy().mult(2)).add(k4v.copy())).mult(1.0 / 6));

  }
  
  PVector calcAcceleration(){
    int i = (int)this.pos.x, j = (int)this.pos.y;
    PVector force = new PVector(0,0);
    PVector displacement = new PVector(0,0);
    
    // Iterating through neighbours
    for(int k = -1; k <= 1; k++){
        for(int l = -1; l <= 1; l++){
            if(i + k < 0 || j + l < 0 || i + k == gridSize || j + l == gridSize) {
                continue;
            }
            
            PVector currDisp = new PVector(point[i + k][j + l].getPos().x, point[i+k][j+l].getPos().y);
            currDisp.sub(point[i][j].getPos());
            
            float distance = currDisp.mag();
            PVector direction = currDisp.normalize();
            
            float dx = distance - springNaturalLength * sqrt(k*k + l*l);
            displacement = direction.mult(dx);
            
            //Shear Spring
            if(l*l + k*k == 2){
                force.add(displacement.x * shearSpringConst, displacement.y * shearSpringConst);    
            }
            //Structure Spring
            else{
                force.add(displacement.x * structSpringConst, displacement.y * structSpringConst); 
            } 
        }
    }
 
    //Damping force
    force.sub(point[i][j].getVel().z * damp, point[i][j].getVel().y * damp);

    return force;
}

};
