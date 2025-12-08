# Data Model: UI & Identity Setup

## Entities

### Brand Identity
- **Name**: Brand Identity
- **Description**: Visual elements including logos, colors, and favicon that represent the fusion of AI and robotics concepts
- **Fields**:
  - `logo_light`: SVG/PNG file for light theme logo
  - `logo_dark`: SVG/PNG file for dark theme logo
  - `favicon`: PNG file for browser tab icon
  - `primary_color`: Tech-Blue hex code for brand color
  - `color_palette`: Object containing all brand colors

### Homepage Content
- **Name**: Homepage Content
- **Description**: Structured content including hero section, value proposition blocks, and call-to-action elements
- **Fields**:
  - `title`: Primary title text ("Physical AI & Humanoid Robotics.")
  - `tagline`: Core tagline text ("Bridging Digital Minds to Physical Bodies.")
  - `primary_cta_text`: Text for primary call to action ("Start Learning (Module 1)")
  - `primary_cta_link`: URL for primary call to action (first module's introduction page)
  - `secondary_cta_text`: Text for secondary call to action ("Chat with Book (Placeholder)")
  - `value_propositions`: Array of 3 value proposition blocks
    - `title`: Title of the value proposition block
    - `content`: Content of the value proposition block
    - `focus`: Focus area (AI-First Pedagogy, 4-Layer Framework, Physical Hardware & Digital Twins)

### Navigation Elements
- **Name**: Navigation Elements
- **Description**: Footer links providing access to documentation, community, and additional resources
- **Fields**:
  - `footer_links`: Array of footer link objects
    - `label`: Display text for the link
    - `url`: Target URL for the link
    - `category`: Category (Docs, Community, More)

## Relationships
- Brand Identity is used by Homepage Content for visual styling
- Homepage Content contains Navigation Elements in the footer
- Brand Identity is referenced in Docusaurus configuration

## State Transitions
- Brand Identity: None (static assets)
- Homepage Content: None (static content)
- Navigation Elements: None (static links)

## Validation Rules
- All image assets must be valid SVG or PNG files
- Primary color must be a valid hex color code
- All URLs must be properly formatted
- Value proposition blocks must contain all required fields