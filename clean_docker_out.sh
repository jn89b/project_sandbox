#remove docker dangling images
docker rmi -f $(docker images -f "dangling=true" -q)

#remove/prune unused docker images
docker image prune 
y
