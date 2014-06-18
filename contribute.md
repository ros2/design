---
layout: default
title: Contribute
show_in_nav: true
---

# Contribute

There are many ways you can contribute to the ROS 2.0 design effort.
The most important of which is probably to review the list of articles posted on the [home page]({{ site.baseurl }}/) and familiarize yourself with the discussions surrounding an issue before getting involved.
This will help keep everyone on the same page and prevent having to discuss things multiple times.

## The Side Bar

Every page on this site has a side bar with some extra information and tools.
In the side bar there are "View Source" and "Edit in Github" buttons, which take you to the source for the page and allow you to edit the page in Github's on-line editor, respectively.

Additionally, if you are logged with Github (see the top right corner of the screen), you will see any open or closed pull requests related to this page.
These pull requests are modifications people have already proposed to this page and should be reviewed before proposing changes of your own.

Finally, a list of contributors to this page are listed if you are logged in.

## Proposing Changes to an Existing Article

If you believe you have something to add to an existing article, then you can click the "Edit in Github" button and make you modifications.
Github will then prompt you for a commit message, which is basically your opportunity to describe what you've changed, and then make a pull request on your behalf.
This is the easiest way to modify an existing article, but you can checkout the "Manually Making a Pull Request" section below if you need to see what it will look like rendered first or if you wish to edit it off-line.

## Submitting a New Article

New articles should be submitted as a new file under the `articles/` folder.
You can add a new file using Github's web interface using this link:

[https://github.com/ros2/design/new/gh-pages/articles](https://github.com/ros2/design/new/gh-pages/articles)

This link will create a new file, give an opportunity to name it, and provide you with an on-line editor to write the file.

New articles should follow this template:

{% raw %}
    ---
    layout: default
    title: Template for New Articles
    abstract:
      This is a multi-line abstract about this article. It should give a good overview about the contents of this article, the reason for writing it, and what the article delivers. The abstract is put in the "front-matter" of the document (YAML between the initial `---`'s) so that it can easily be reused else where in the site.
    author: '[William Woodall](https://github.com/wjwwood)'
    published: false
    ---

    * This will become a table of contents (this text will be scraped).
    {:toc}

    # {{ page.title }}

    <div class="abstract" markdown="1">
    {{ page.abstract }}
    </div>

    Original Author: {{ page.author }}

    ## First Heading

    Content....

{% endraw %}

## Manually Making a Pull Request

If you need to reproduce the site locally and/or make extensive changes, you can clone the entire website and work on it locally before making a pull request manually.

### Forking this Repository

First check to see if you already have a fork of this repository:

{% raw %}
    https://github.com/<your username>/design
{% endraw %}

If you do not, then browse to this [repository](https://github.com/ros2/design) site and click on the "Fork" button:

<img style="height: 50px;" src="{{ site.baseurl }}/img/fork.png"/>

Github will now go off and create a fork off this repository into your Github account.
Then you can clone your fork of the repository by running this command:

    git clone https://github.com/<your username>/design.git

This will clone this repository onto your machine into a folder called `design`. You can edit these files in this folder using your favorite editor.

### Using Jekyll to Preview Changes

If your change is more extensive or you want to see what your changes will look like on the live site, you can do so by running Jekyll locally.
First install Jekyll for your system:

[http://jekyllrb.com/docs/installation/](http://jekyllrb.com/docs/installation/)

Once you have Jekyll installed you should be able to run this command:


    jekyll serve --watch --baseurl=''


The `jekyll server` command will start a web server which you can access at `http://localhost:4000`.
The `--watch` option will cause the jekyll web server to regenerate pages which are changed each time they are modified.
This allows you to make quick edits to the documents and then refresh the web page to get the changes immediately.
The `--baseurl=''` option sets the `baseurl` variable for the site's global config, allowing static files like the `css` and `js` files work on your `localhost`.

### Submitting a Pull Request

Once you are satisfied with your changes you can follow the Github tutorial on [creating a pull request](https://help.github.com/articles/creating-a-pull-request) against ours, where we can review and discuss your changes before merging.
