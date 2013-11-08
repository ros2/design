require 'jekyll'
require 'jekyll/page'

class Jekyll::Page
  alias_method :original_to_liquid, :to_liquid

  def to_liquid
    original_to_liquid.deep_merge({
      'filename' => @name
    })
  end
end
