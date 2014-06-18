$( document ).ready(function() {
    if (!window.location.origin) {
        window.location.origin = window.location.protocol+"//"+window.location.host;
    }
    // If already logged in, go to home page
    if (localStorage.getItem('github_oauth_token')) {
        window.location.replace(target_url);
    }
    // Check for redirect_uri
    var target_url = sessionStorage.getItem('login_redirect');
    if (!target_url) {
        target_url = window.location.origin;
    } else {
        sessionStorage.removeItem('login_redirect');
    }
    // Get temporary code from HTTP GET
    var match = window.location.href.match(/\?code=(.*)/)[1];
    // If found, pass the temp token to gatekeeper
    if (match) {
        var code = match[1];
        $.getJSON('http://auth.design.ros2.org/authenticate/' + code, function(data) {
            // Once gatekeeper gets back, check for the token
            if (!data.token) {
                // If the token is not set, then display an error
                $( 'div.content-container div' ).replaceWith([
                    '<div class="panel panel-danger">',
                    '  <div class="panel-heading">',
                    '    <h3 class="panel-title">Error</h3>',
                    '  </div>',
                    '  <div class="panel-body">',
                    '    Error logging in using Github: ' + data.error,
                    '  </div>',
                    '</div>'].join('\n'));
            } else {
                // Else store the key in the the localStorage
                localStorage.setItem('github_oauth_token', data.token);
                window.location.replace(target_url);
            }
        });
    } else {
        // If no match return the main page
        window.location.replace(window.location.origin);
    }
});
