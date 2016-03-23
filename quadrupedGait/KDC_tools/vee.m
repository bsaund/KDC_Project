function out = vee(in)
% takes the skew components into a vector
if size(in,2)==4
out = [in(1:3,4);...
       in(3,2); in(1,3); in(2,1)];
%  S(n) = [0 -n(3) n(2) ;
%          n(3) 0 -n(1) ;
%         -n(2) n(1) 0 ];
else
    out = [in(3,2); in(1,3); in(2,1)];
end

end
