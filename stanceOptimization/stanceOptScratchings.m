% optimize the stance so that it can lift a leg up without tipping.

% find the angles th that minimize the distance from the COM to the
% support centroid, while staying within joint limits.

criterion:
(xyCOM - xyCentroid) - wont work because would just move feet to center
Distance from all the feet

linear constraint:
th<pi, th>-pi

non-linear contstraint:
xyCOM is in polygon

initial stance: all legs zero
thLeg1 = [pi/2; 0; pi/2]; %reaching out far for worst case
th0 = zeros(3,5); % leg one angles will be held fixed


xyCOM = getCOM(th)
xyCent= getCentroid(th)
