target extended-remote :3333

define cll
make
target extended-remote :3333
load
monitor reset init
end

define mri
monitor reset init
end
