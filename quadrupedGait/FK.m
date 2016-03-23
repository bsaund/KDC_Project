function [gt, gl2, gl3, gl4] = FK(SMData, th)

% FK map
gst = SMData.gst_0;
gt = zeros(size(gst));
gl2 = gt; gl3 = gt; gl4 = gt;
gsl2 = SMData.gsl2_0 ; % the poses of joints 2 in leg base frame
gsl3 = SMData.gsl3_0 ; % the poses of joints 3 in leg base frame
gsl4 = SMData.gsl4_0 ; % the poses of joints 3 in leg base frame

for leg = 1:6       
%         for j = 3:-1:1
%             gst(:,:,leg) = twistExp(SMData.xi(:,j,leg),th(j,leg))*gst(:,:,leg);
%         end
            gst(:,:,leg) = twistExp(SMData.xi(:,1,leg),th(1,leg))...
                          *twistExp(SMData.xi(:,2,leg),th(2,leg))...
                          *twistExp(SMData.xi(:,3,leg),th(3,leg))*gst(:,:,leg);
            gsl2(:,:,leg) = twistExp(SMData.xi(:,1,leg),th(1,leg))*gsl2(:,:,leg);
            gsl3(:,:,leg) = twistExp(SMData.xi(:,1,leg),th(1,leg))...
                          *twistExp(SMData.xi(:,2,leg),th(2,leg))*gsl3(:,:,leg);
            gsl4(:,:,leg) = twistExp(SMData.xi(:,1,leg),th(1,leg))...
                          *twistExp(SMData.xi(:,2,leg),th(2,leg))...
                          *twistExp(SMData.xi(:,3,leg),th(3,leg))*gsl4(:,:,leg);
                      
% rotate to align with chassis
        gt(:,:,leg) = SMData.gs(:,:,leg)*gst(:,:,leg); 
        gl2(:,:,leg) = SMData.gs(:,:,leg)*gsl2(:,:,leg); 
        gl3(:,:,leg) = SMData.gs(:,:,leg)*gsl3(:,:,leg); 
        gl4(:,:,leg) = SMData.gs(:,:,leg)*gsl4(:,:,leg); 
        
end

end