function mask = isOdd(full)
% returns the logical mask of the odd values in the array
mask = mod(full,2)==1;

end