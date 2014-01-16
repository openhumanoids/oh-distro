# Set up the git authorship functions
# Source this file in your .bashrc or equivalent to set up the functions

# Sets git authorship by looking up your username in $AUTHORS_FILE
function set-git-author() {
    AUTHORS_FILE=$DRC_BASE/software/config/git_config/drc-authors-transform.txt
    if cat $AUTHORS_FILE | grep -q $1
    then
        export GIT_AUTHOR_NAME="`cat $AUTHORS_FILE | grep $1 | sed 's/.*=\s*//' | sed 's/\s*<.*$//'`"
        export GIT_AUTHOR_EMAIL=`cat $AUTHORS_FILE | grep $1 | sed 's/^.*<//' | sed 's/>.*$//'`
        echo "Welcome, $1"
        echo "Your git authorship has been automatically set as follows:"
        git var GIT_AUTHOR_IDENT
        echo 'This will last until this terminal is closed, or until you run "clear-git-author"'
    else
        echo "Error: User $1 cannot be found in $AUTHORS_FILE. If you are a new user, please add an entry for yourself in that file, using the format \"yourusername = Firstname Lastname <email@host.com>\""
    fi
}

# Unsets git authorship, to reverse the changes in set-git-author
function clear-git-author() {
    unset GIT_AUTHOR_EMAIL
    unset GIT_AUTHOR_NAME
	echo "Goodbye. Git authorship reset to default values:"
	git var GIT_AUTHOR_IDENT
}
