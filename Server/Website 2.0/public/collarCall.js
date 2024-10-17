function test(buttonName){
    fetch("/alarm", {
        method: "POST",
        body: JSON.stringify({
            button: buttonName
        }),
        headers: {
            'Content-Type': 'application/json'       
        }
        }).catch(err => {
            console.log(err);
        })
}