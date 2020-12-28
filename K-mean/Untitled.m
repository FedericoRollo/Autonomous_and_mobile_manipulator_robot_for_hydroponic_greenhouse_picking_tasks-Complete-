%clear all the variables and figures 
clc; clear all; close all;

%Load the image for the segmantation
zucc = imread('zucc.jpg');
imshow(zucc);

%move the image from the RGB space to the CIE XYZ (Lab) one and extract the color
%plane
xyz_zucc = rgb2lab(zucc);
xz = xyz_zucc(:,:,2:3);
xz = im2single(xz);

% number of different colors to be extracted. It characterize the number of
% the final clusters
n_Clusters = 3;

%k-mean segmantation
pixel_labels = imsegkmeans(xz,n_Clusters,'NumAttempt',3);

%shows the clusters labelled with different grays colors
figure
imshow(pixel_labels,[]) 

%use different masks to extract form the original image the clusters found.
mask1 = pixel_labels==1;
cluster1 = zucc .* uint8(mask1);
figure
imshow(cluster1)

mask2 = pixel_labels==2;
cluster2 = zucc .* uint8(mask2);
figure
imshow(cluster2)

mask3 = pixel_labels==3;
cluster3 = zucc .* uint8(mask3);
figure
imshow(cluster3)