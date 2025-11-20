function [phiR_max, phiL_max, Rp] = getMaxBankAngle(phi_nom, daLim, daSurf, isLeftStuck)
% Calculates the maximum bank angle for left/right turn when an aileron is
% jammed. 
% Based on "Evaluation of Reduction in the Performance of a Small UAV After 
% an Aileron Failure for an Adaptive Guidance System," 
% doi: 10.1109/ACC.2007.4282845.
    
    da_max = daLim(1);
    da_min = daLim(2);
    
    % Roll-authority calculation
    % Calculates the aileron deflection for max roll torque to the right and
    % left when the aileron is stuck
    if isLeftStuck                     % LEFT aileron jammed
        daL = daSurf(1);               
        dR  = 0.5*( da_max - daL );
        dL  = 0.5*( da_min - daL );
    else                               % RIGHT aileron jammed
        daR = daSurf(2);               
        dR  = 0.5*( daR - da_min );
        dL  = 0.5*( daR - da_max );
    end
    
    Rp = abs(dL) / abs(dR);            %  if > 1, left roll is stronger
    
    % Bank angle limits for equal roll-out time
    % Keeps the weak side at nominal max bank angle and limits the strong side
    if abs(dL) > abs(dR)               % LEFT roll stronger  (Rp > 1)
        phiR_max = +phi_nom;
        phiL_max = -phi_nom / Rp;
    else                               % RIGHT roll stronger (Rp < 1)
        phiL_max = -phi_nom;
        phiR_max = +phi_nom * Rp;
    end
end
