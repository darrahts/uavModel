function img = helperPlotImage(data)
%helperPlotImage converts an uint8 vector into a 240*320*3 matrix
%representing the image with RGB channels.

width = 320;
height = 240;
numOutputChannels = 3;
indexBase = (1:numOutputChannels:width*height*numOutputChannels);
index = zeros(numel(indexBase),numOutputChannels);
for i = 1:numOutputChannels
    % Construct it as column-major index to ease later reshape
    index(:, i) = indexBase + i - 1;
end

% Reshape the data into a width x height x numOutputChannels matrix
img = reshape(data(index), width, height, numOutputChannels);

% Since reshape works column-wise, bring image back to
% row-major order (transpose of 1st and 2nd dimension
% Result will be a height x width x numOutputChannels image matrix
img = permute(img, [2 1 3]);
end

