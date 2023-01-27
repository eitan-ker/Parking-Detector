function updateDynamicValue() {
          var xhr = new XMLHttpRequest();
          xhr.open('GET', '/info', true);
          xhr.onload = function() {
              if (xhr.status === 200) {
                  let data = JSON.parse(xhr.responseText)
                  document.getElementById("total_spaces").innerHTML = data.totalSpaces_value;
                  document.getElementById("free_spaces").innerHTML = data.freeSpaces_value;

              }
          };
          xhr.send();
        }
setInterval(updateDynamicValue, 100);