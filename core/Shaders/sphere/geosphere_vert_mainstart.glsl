void main(void) {
    
    gl_Position = inVertex;
    float rad = inVertex.w;
    if (CONSTRAD > -0.5) {
      gl_Position.w = CONSTRAD;
      rad = CONSTRAD;
    }
    
    // clipping
    if (any(notEqual(clipDat.xyz, vec3(0, 0, 0)))) {
        float od = dot(inVertex.xyz, clipDat.xyz) - rad;
        if (od > clipDat.w) {
          gl_Position = vec4(1.0, 1.0, 1.0, 0.0);
        }
    }   
