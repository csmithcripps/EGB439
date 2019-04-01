function out = myFunction(op, varargin)
    switch op
        case 'window'
            out = window(varargin{:});
        case 'minwin'
            out = minwin(varargin{:});
        case 'minval'
            out = minval(varargin{:});
    end
end

function M = window(A, x, y) 
% Input:
%  A an arbitrary sized real matrix, at least 3x3.
%  x the x-coordinate (horizontal index) of the centre of the window.
%  y the y-coordinate (vertical index) of the centre of the window.
% Return:
%  M as 3x3 matrix which are the elements of M centered on the element (x,y).
%
% Should the window extend beyond the edges of the matrix the function must
% return an empty matrix [].
    
    % Test Values
    sizes = size(A);

    if x < sizes(2) && x > 1 && y < sizes(1) && y > 1
        M = A(y-1:y+1,x-1:x+1);
    else
        M = [];
    end
end

function B = minwin(A) 
% Input:
%  A returns a matrix the same size as A where each element of B is the minimum 
% of a 3x3 window of A centered at the corresponding element.  Elements of B 
% that correspond to a window which "falls off the edge" of the matrix A should be set to a value of NaN.
        
    sizes = size(A)
    B = zeros(sizes)
    
    
    for x = 1:sizes(2)
        for y = 1:sizes(1)
            M = window(A,x,y);
            if isempty(M)
                B(y,x) = NaN;
            else
                B(y,x) = min(min(M));
            end
        end
    end
end

function next = minval(M)
% Input:
%  M is a real 3x3 matrix
% Return:
%  next is a 1x2 matrix with elements [x, y] which are the horizontal and vertical coordinates relative to the centre
%       element, of the smallest element in the matrix.
    [row,col] = find(M == min(min(M)))
    next = fliplr([row,col] - [2,2]);
end
