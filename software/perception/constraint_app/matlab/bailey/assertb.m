function assertb(val, message)
%function assert(val, message)
%
% Simple assertion function to check logical invariants. Useful as a
% debugging tool. Prints message and aborts if val==0.
if any(val(:) == 0)
    error(message)
end
