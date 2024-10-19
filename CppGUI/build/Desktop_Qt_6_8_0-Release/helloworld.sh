!/bin/bash
# Shell script to open terminals
# and execute a separate command in each

# Commands to run (one per terminal)
cmds=('echo 'hello1'')

# Loop through commands, open terminal, execute command
for i in "${cmds[@]}"
do
    gnome-terminal -- sh -c "$i && /bin/bash" &
    
done
echo "I just opened ${#cmds[@]} terminals!"
