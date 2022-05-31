function timestamp() {
    var time = new Date(),
    year = time.getFullYear(),
    month = time.getMonth()+1,
    day = time.getDate(),
    hours = time.getHours(),
    minutes = time.getMinutes(),
    seconds = time.getSeconds();
    document.querySelectorAll('.clock')[0].textContent = harold(day) + "-" + harold(month) + "-" + year + " " + harold(hours) + ":" + harold(minutes) + ":" + harold(seconds);

    function harold(standIn) {
        if (standIn < 10) standIn = '0' + standIn
        return standIn;
    }
}
setInterval(timestamp, 1000);
