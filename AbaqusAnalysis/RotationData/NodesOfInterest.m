function activeNodes = NodesOfInterest(filename)

% This function extracts the node numbersof a file with a given
% lay-out. The current set up allows to extract node numbers that in
% Abaqus/CAE have been obtained by:
% 1) Options>Common...>Label>"Show Note Labels (check box)">Apply/OK
% 2) Tools>View Cut> Manager... (use menu to ONLY display part of interest)
% 3) Use cursor drag a box over the part of interest
% 4) Press "Replace Selected" from the top menu (indicated by two overlapping "O"s, color left one white,right one orange)
% 5a) Report>Field Output...>(sub menu) Setup >(change file name)
% 5b) Report>Field Output...>(sub menu) Postion > Unique Nodal>
% "U:spatial displacement (check box)" > OK

fid = fopen(filename,'rt');
S = textscan(fid,'%s','Delimiter','\n');
S = S{1} ;
%%Get the line number 
idxNode = strfind(S, '--------------------');  % search the text for this expression
idx1 = find(not(cellfun('isempty', idxNode))); % find index in which previous expression is found
idxNodeEnd = strfind(S, 'Minimum');            % search the text for this expression
idx2 = find(not(cellfun('isempty', idxNodeEnd)));% find index in which previous expression is found
% pick  nodes 
nodes = S(idx1+1:idx2-3);                      % between those 2 expression node numbers are given
Nodes = zeros(length(nodes),5);                % pre-allocate
for ii = 1:length(nodes)
    Nodes(ii,:) = str2num(nodes{ii});          % write data in matrix
end
activeNodes = Nodes(:,1);                      % make a collum of active nodes


