function X = tanksmodel(~, x, u, l1, l2,vars)
    
    a2 = vars(1);
    a3 = vars(2);
    A1 = vars(3);
    A2 = vars(4);
    q0 = vars(5);
    g = vars(6);
         
    u1 = u(1);
    u2 = u(2);
    u3 = u(3);
    h1 = x(1);
    h2 = x(2); 
    
    if ( h1 < 0 )
        h1 = 0;
    end
    
    if ( h2 < 0 )
        h2 = 0;
    end
        
    dh1 = (1/A1)*( u1*q0 - l1 - u2*a2*sqrt(2*g*(h1)) );
    dh2 = (1/A2)*( u2*a2*sqrt(2*g*(h1)) - l2 - u3*a3*sqrt(2*g*h2) );

    X = [dh1; dh2];   
    