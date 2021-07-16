function CD = CDcalc(vehicleLength,noseLength, nFins, finSpan, finThickness, finRootChord, finTipChord, c, airspeed, rho, mu, alpha, beta)
% Calculates the drag at any given moment. Assumes consistent body
% diameter. Would need to be updated for different diameter sections/boat
% tail
%   Detailed explanation goes here


lm = (finRootChord + finTipChord)/2;

% calculate reynolds numbers
Reb = (rho * airspeed * vehicleLength)/mu;
Ref = (rho * airspeed * lm)/mu;
if (Reb <= 0)
    CD = 0;
else
    
    if (Reb <= 5e5)
        Cffb = 1.328/sqrt(Reb);
    else
        B = (5e5)*(0.074/(Reb^(1/5))-1.328/sqrt(Reb));
        Cffb = (0.074/(Reb^(1/5))) - B/Reb;
    end
    
    if (Ref <= 5e5)
        Cff = 1.328/sqrt(Ref);
    else
        B = (5e5)*(0.074/(Ref^(1/5))-1.328/sqrt(Ref));
        Cff = (0.074/(Ref^(1/5))) - B/Ref;
    end
    
    
    % zero angle of attack
    lb = vehicleLength - noseLength;
    CDfb = (1 + (60/((vehicleLength/c)^3)) + 0.0025*(lb/c)) * (2.7*(noseLength/c) + 4 * (lb/c)) * Cffb; % body drag
    CDb = 0.029 * (1/sqrt(CDfb)); % base drag
    Afe = 0.5 * (finRootChord + finTipChord) * finSpan;
    Afp = Afe + 0.5 * c * finRootChord;
    CDf = 2*Cff * (1 + 2 * (finThickness/lm)) * ((4 * nFins * Afp)/(pi * c^2));
    CDi = 2* Cff * (1 + 2 * (finThickness/lm)) * ((4*nFins*(Afp-Afe))/(pi * c^2));
    
    CD0 = CDfb + CDb + CDf + CDi;
    
    % angle of attack drag (only valid for certain regimes
    lts = 2 * finSpan + c;
    Rs = lts/c;
    kfb = 0.8065*Rs^2 + 1.1553*Rs;
    kbf = 0.1935*Rs^2 + 0.8174*Rs +1;
    CDa = (alpha^2) * (1.2*((4*Afp)/(pi*c^2)) + 3.12*(kfb + kbf -1)*((4 * Afe)/(pi*c^2)));
    CDb = (beta^2) * (1.2*((4*Afp)/(pi*c^2)) + 3.12*(kfb + kbf -1)*((4 * Afe)/(pi*c^2)));
    
    % total drag (account for AoA limits
    if(alpha >= deg2rad(10) || alpha <= -deg2rad(10))
        CDa = 0;
    end
    
     if(beta >= deg2rad(10) || beta <= -deg2rad(10))
        CDb = 0;
    end
    
        
        
        
    CD = CD0 + CDa + CDb;
end

end

