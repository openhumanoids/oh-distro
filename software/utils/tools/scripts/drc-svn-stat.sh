#bash
svn stat | grep -v "Performing status" | grep -v "X     "  | grep -v '^$'
