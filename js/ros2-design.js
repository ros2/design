$( document ).ready(function() {
    // Add table of contents title to toc generated from markdown pages
    $( 'div.maruku_toc' ).prepend('<p><strong>Table of Contents</strong></p>');
    // Add the bootstrap table class to table's inside of div's with table class
    $( 'div.table table' ).addClass('table').addClass('table-striped');
});