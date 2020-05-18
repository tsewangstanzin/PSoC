void profiling(int destination)
 { 
  int distance =   destination - setpoint; 
  int finalPoint = setpoint + distance;
  boolean dirPos = true;
  if (distance < 0) {
    distance = -distance;  
    dirPos = false;
  }
  float xm = feed * feed / accel;   //critical distance
  float t1, t2;
  
  
  if (distance <= xm)   // triangular
	  t1 = t2 = sqrt(distance / accel);
  else                 // trapezoidal
  { 
	
    t1 = sqrt(xm / accel); 					
    t2 = (distance - xm) / feed + t1; 		
  }
  // Performing the actual motion
  
  float t = 0, spd = 0.0;
  float dt = 1e-3;
  float da = accel * dt;
  float covered = setpoint;
  float maxt = t1 + t2;
  while (t < maxt) {
    t += dt;
    if (t < t1) spd += da; else if (t >= t2) spd -= da;
    if ( dirPos ) covered += spd * dt; else covered -= spd * dt; // calculate new target position
    //vel =  encoder0Pos - input;
    input = encoder0Pos;
    setpoint = covered;
    while(!myPID.Compute()); // Wait untill PID computes
    setspeed = output;
    //speed.Compute();
    pwmOut(output );
    if (counting  ) {
      pos[p] = encoder0Pos;
      if (p < 999) p++;
      else counting = false;
    }
  }
  int Error=encoder0Pos - covered;
  target1 = covered;
}

