% this wrapper function allows the assessment code to access your functions
function [out1,out2] = myFunction(op, varargin)
    switch op
        case 'qappend'
            out1 = Astar1.qappend(varargin{:});
        case 'qcontains'
            out1 = Astar1.qcontains(varargin{:});
        case 'qpop'
            [out1,out2] = Astar1.qpop(varargin{:});
        case 'qinsert'
            out1 = Astar1.qinsert(varargin{:});
        case 'neighbours'
            out1 = Astar1.neighbours(varargin{:});            
    end
end