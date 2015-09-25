for file in *Left.*; do
    #mv "$file" "${file/cmpsms*2014*.p/cmpsms25032014*.p}"
    echo $file
    mv $file ${file,,}
done


for file in *Right.*; do
    #mv "$file" "${file/cmpsms*2014*.p/cmpsms25032014*.p}"
    echo $file
    mv $file ${file,,}
done
