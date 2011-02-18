function pp = pchipDeriv(t,y,ydot_minus,ydot_plus)

% implements piecewise cubic Hermite polynomials with the derivatives
% specified.  (e.g., as in Hargraves86).

if (nargin<4) ydot_plus = ydot_minus; end

n = size(y,1);
if (length(size(y))~=2) error('only handle vectors for now'); end
if (size(y,2)~=length(t)) error('y should be the same size as t'); end

if (size(ydot_minus,2)>=length(t)) ydot_minus = ydot_minus(:,2:end); end
if (size(ydot_minus,2)~=(length(t)-1)) error('ydot_minus is the wrong size'); end

if (size(ydot_plus,2)>=length(t)) ydot_plus = ydot_plus(:,1:(end-1)); end
if (size(ydot_plus,2)~=(length(t)-1)) error('ydot_plus is the wrong size'); end

dt = diff(t);

for i=1:(length(t)-1)
  % solve the following vector equations:
  %   y(:,i) = c4
  %   y(:,i+1) = c4 + dt*c3 + dt^2*c2 + dt^3*c1
  %   ydot_plus(:,i) = c3
  %   ydot_minus(:,i) = c3 + 2*dt*c2 + 3*dt^2*c1  (not i+1, because I shifted it above) 
  % rewritten as 
  %   [  0  0  0  I ] [ c1; c2; c3; c4 ] = y(:,i) , etc... 

  a=dt(i); b=a^2; c=a^3;
  coefs(:,i,:) = reshape(inv([zeros(n,3*n),eye(n); ...
    c*eye(n),b*eye(n),a*eye(n),eye(n); ...
    zeros(n,2*n),eye(n),zeros(n); ...
    3*b*eye(n),2*a*eye(n),eye(n),zeros(n)]) * ...
    [y(:,i);y(:,i+1);ydot_plus(:,i);ydot_minus(:,i)],n,4);
end

pp = mkpp(t,coefs,n);
pp.dim = n;